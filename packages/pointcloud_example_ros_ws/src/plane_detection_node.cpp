#include "plane_detection_node.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

PlaneDetectionNode::PlaneDetectionNode(ros::NodeHandle &nh) : nh(nh)
{
    nh.param<std::string>("target_frame", target_frame, "base_link");
    nh.param<double>("passthrough_x_min", passthrough_x_min, 0.0);
    nh.param<double>("passthrough_x_max", passthrough_x_max, 8.0);
    nh.param<double>("passthrough_y_min", passthrough_y_min, -0.7);
    nh.param<double>("passthrough_y_max", passthrough_y_max, 0.7);
    nh.param<double>("voxel_leaf_size", voxel_leaf_size, 0.01);
    nh.param<double>("cluster_tolerance", cluster_tolerance, 0.02);
    nh.param<int>("min_cluster_size", min_cluster_size, 50);
    nh.param<int>("max_cluster_size", max_cluster_size, 10000);
    tf_listener.reset(new tf::TransformListener);

    cluster_extraction.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());

    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setRadiusSearch(0.10);

    pointcloud_subscriber = nh.subscribe("input_pointcloud_topic", 1, &PlaneDetectionNode::pointcloudCallback, this);
    pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_pointcloud_topic", 1);
    plane_polygon_publisher = nh.advertise<geometry_msgs::PolygonStamped>("output_plane_polygon", 1);
    pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("object_pose", 1);
}

PlaneDetectionNode::~PlaneDetectionNode()
{
}

void PlaneDetectionNode::pointcloudCallback(const sensor_msgs::PointCloud2Ptr &cloud_in)
{
    sensor_msgs::PointCloud2 cloud_in_transformed;
    try
    {
        ros::Time common_time;
        tf_listener->getLatestCommonTime(target_frame, cloud_in->header.frame_id, common_time, NULL);
        cloud_in->header.stamp = common_time;
        tf_listener->waitForTransform(target_frame, cloud_in->header.frame_id, ros::Time::now(),
                                      ros::Duration(1.0));
        pcl_ros::transformPointCloud(target_frame, *cloud_in, cloud_in_transformed, *tf_listener);
        cloud_in_transformed.header.frame_id = target_frame;
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("PCL transform error: %s", ex.what());
        return;
    }
    PointCloud::Ptr full_cloud(new PointCloud);
    pcl::fromROSMsg(cloud_in_transformed, *full_cloud);

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PlanarPolygon<PointT>::Ptr hull_polygon(new pcl::PlanarPolygon<PointT>);
    PointCloud::Ptr hull_pointcloud(new PointCloud);

    PointCloud::Ptr plane = getPlane(full_cloud, Eigen::Vector3f(0.0, 0.0, 1.0), plane_coefficients, hull_polygon, hull_pointcloud);

    geometry_msgs::PolygonStamped poly;
    poly.header.stamp = ros::Time::now();
    poly.header.frame_id = cloud_in_transformed.header.frame_id;

    for (int i = 0; i < hull_pointcloud->size(); i++)
    {
        geometry_msgs::Point32 pt;
        pt.x = hull_pointcloud->points[i].x;
        pt.y = hull_pointcloud->points[i].y;
        pt.z = hull_pointcloud->points[i].z;
        poly.polygon.points.push_back(pt);
    }
    if (poly.polygon.points.size() > 0)
    {
        plane_polygon_publisher.publish(poly);
    }

    pcl::PointIndices::Ptr segmented_cloud_inliers(new pcl::PointIndices);

    extract_polygonal_prism.setHeightLimits(0.01, 0.2);
    extract_polygonal_prism.setInputPlanarHull(hull_pointcloud);
    extract_polygonal_prism.setInputCloud(full_cloud);
    extract_polygonal_prism.setViewPoint(0.0, 0.0, 2.0);
    extract_polygonal_prism.segment(*segmented_cloud_inliers);

    std::vector<pcl::PointIndices> clusters_indices;
    std::vector<PointCloud::Ptr> clusters;

    cluster_extraction.setInputCloud(full_cloud);
    cluster_extraction.setIndices(segmented_cloud_inliers);
    cluster_extraction.setClusterTolerance(cluster_tolerance);
    cluster_extraction.setMinClusterSize(min_cluster_size);
    cluster_extraction.setMaxClusterSize(max_cluster_size);


    cluster_extraction.extract(clusters_indices);

    int center_most_cluster_id = 0;
    double dist_to_center = 10.0;


    for (size_t i = 0; i < clusters_indices.size(); i++)
    {
        const pcl::PointIndices& cluster_indices = clusters_indices[i];
        PointCloud::Ptr cluster(new PointCloud);
        pcl::copyPointCloud(*full_cloud, cluster_indices, *cluster);
        clusters.push_back(cluster);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        if (centroid[1] < dist_to_center)
        {
            dist_to_center = std::fabs(centroid[1]);
            center_most_cluster_id = i;
        }
    }

    if (!clusters.empty())
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*(clusters[center_most_cluster_id]), centroid);

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = cloud_in_transformed.header.frame_id;
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = centroid[0];
        pose.pose.position.y = centroid[1];
        pose.pose.position.z = centroid[2];
        pose.pose.orientation.w = 1.0;
        pose_publisher.publish(pose);
    }
    /* publish debug pointcloud */
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*plane, output);
    if (!clusters.empty())
    {
        pcl::toROSMsg(*(clusters[center_most_cluster_id]), output);
    }
    output.header.frame_id = cloud_in_transformed.header.frame_id;
    output.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(output);
}

PointCloud::Ptr PlaneDetectionNode::getPlane(const PointCloud::Ptr &full_cloud, const Eigen::Vector3f &axis,
                    pcl::ModelCoefficients::Ptr &coefficients, pcl::PlanarPolygon<PointT>::Ptr &hull_polygon,
                    PointCloud::Ptr &hull_pointcloud)
{
    PointCloud::Ptr filtered_cloud(new PointCloud);
    passthrough_filter.setInputCloud(full_cloud);
    passthrough_filter.setFilterFieldName("x");
    passthrough_filter.setFilterLimits(passthrough_x_min, passthrough_x_max);
    passthrough_filter.filter(*filtered_cloud);

    passthrough_filter.setInputCloud(filtered_cloud);
    passthrough_filter.setFilterFieldName("y");
    passthrough_filter.setFilterLimits(passthrough_y_min, passthrough_y_max);
    passthrough_filter.filter(*filtered_cloud);

    voxel_filter.setInputCloud(filtered_cloud);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*filtered_cloud);

    // indices of points which are part of the detected plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    PointCloudN::Ptr normals(new PointCloudN);
    normal_estimation.setInputCloud(filtered_cloud);
    normal_estimation.compute(*normals);

    sac_segmentation.setAxis(axis);
    sac_segmentation.setInputNormals(normals);
    sac_segmentation.setInputCloud(filtered_cloud);
    sac_segmentation.setOptimizeCoefficients(true);
    sac_segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);
    sac_segmentation.setDistanceThreshold(0.05);
    sac_segmentation.setEpsAngle(0.3);

    sac_segmentation.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cout << "could not find plane " << std::endl;
        return full_cloud;
    }

    PointCloud::Ptr plane_cloud(new PointCloud);

    project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    project_inliers.setInputCloud(filtered_cloud);
    project_inliers.setIndices(inliers);
    project_inliers.setModelCoefficients(coefficients);
    project_inliers.filter(*plane_cloud);

    convex_hull.setInputCloud(plane_cloud);
    convex_hull.reconstruct(*hull_pointcloud);
    hull_pointcloud->points.push_back(hull_pointcloud->points.front());
    hull_pointcloud->width += 1;
    hull_polygon->setContour(*hull_pointcloud);
    hull_polygon->setCoefficients(*coefficients);

    return plane_cloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detect_plane_node");

    ros::NodeHandle n("~");

    PlaneDetectionNode pointcloud_writer_node(n);

    ros::spin();

    return 0;
}
