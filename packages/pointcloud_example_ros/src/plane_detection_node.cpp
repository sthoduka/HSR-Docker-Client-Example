#include "plane_detection_node.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

PlaneDetectionNode::PlaneDetectionNode(ros::NodeHandle &nh) : nh(nh)
{
    tf_listener.reset(new tf::TransformListener);

    nh.param<std::string>("target_frame", target_frame, "base_link");
    pointcloud_subscriber = nh.subscribe("input_pointcloud_topic", 1, &PlaneDetectionNode::pointcloudCallback, this);
    pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("output_pointcloud_topic", 1);
    plane_polygon_publisher = nh.advertise<geometry_msgs::PolygonStamped>("output_plane_polygon", 1);

    normal_estimation.setSearchMethod(boost::make_shared<pcl::search::KdTree<PointT> >());
    normal_estimation.setRadiusSearch(0.1);
    this->voxel_leaf_size = 0.01;
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

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*plane, output);
    output.header.frame_id = cloud_in_transformed.header.frame_id;
    output.header.stamp = ros::Time::now();
    pointcloud_publisher.publish(output);
}

PointCloud::Ptr PlaneDetectionNode::getPlane(const PointCloud::Ptr &full_cloud, const Eigen::Vector3f &axis,
                    pcl::ModelCoefficients::Ptr &coefficients, pcl::PlanarPolygon<PointT>::Ptr &hull_polygon,
                    PointCloud::Ptr &hull_pointcloud)
{
    PointCloud::Ptr filtered_cloud(new PointCloud);
    voxel_filter.setInputCloud(full_cloud);
    voxel_filter.setLeafSize(this->voxel_leaf_size, this->voxel_leaf_size, this->voxel_leaf_size);
    voxel_filter.filter(*filtered_cloud);

    PointCloudN::Ptr normals(new PointCloudN);
    normal_estimation.setInputCloud(filtered_cloud);
    normal_estimation.compute(*normals);

    // indices of points which are part of the detected plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    sac_segmentation.setAxis(axis);
    sac_segmentation.setInputNormals(normals);

    sac_segmentation.setOptimizeCoefficients(true);
    sac_segmentation.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    sac_segmentation.setMethodType(pcl::SAC_RANSAC);

    sac_segmentation.setDistanceThreshold(0.05);
    sac_segmentation.setEpsAngle(0.3);

    sac_segmentation.setInputCloud(filtered_cloud);
    sac_segmentation.segment(*inliers, *coefficients);

    PointCloud::Ptr plane_cloud(new PointCloud);

    project_inliers.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    project_inliers.setInputCloud(filtered_cloud);
    project_inliers.setIndices(inliers);
    project_inliers.setModelCoefficients(coefficients);

    project_inliers.filter(*plane_cloud);

    convex_hull.setInputCloud(filtered_cloud);
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

