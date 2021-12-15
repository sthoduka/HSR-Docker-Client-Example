#ifndef PLANE_DETECTION_NODE_H_
#define PLANE_DETECTION_NODE_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>


#include <pcl/common/common.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/filters/extract_indices.h>
#include <string>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::Normal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudN;


class PlaneDetectionNode
{
public:
    PlaneDetectionNode(ros::NodeHandle &nh);
    virtual ~PlaneDetectionNode();

    void pointcloudCallback(const sensor_msgs::PointCloud2Ptr &cloud_in);

private:
    /**
     * ROS NodeHandle object
     */
    ros::NodeHandle nh;

    ros::Subscriber pointcloud_subscriber;
    ros::Publisher pointcloud_publisher;
    ros::Publisher plane_polygon_publisher;
    ros::Publisher pose_publisher;
    boost::shared_ptr<tf::TransformListener> tf_listener;
    std::string target_frame;

    double passthrough_x_min;
    double passthrough_x_max;
    double passthrough_y_min;
    double passthrough_y_max;
    double cluster_tolerance;
    double voxel_leaf_size;
    int min_cluster_size;
    int max_cluster_size;
    pcl::PassThrough<PointT> passthrough_filter;
    pcl::VoxelGrid<PointT> voxel_filter;
    pcl::SACSegmentationFromNormals<PointT, PointNT> sac_segmentation;
    pcl::ProjectInliers<PointT> project_inliers;
    pcl::EuclideanClusterExtraction<PointT> cluster_extraction;
    pcl::NormalEstimation<PointT, PointNT> normal_estimation;
    pcl::ConvexHull<PointT> convex_hull;
    pcl::ExtractPolygonalPrismData<PointT> extract_polygonal_prism;

    PointCloud::Ptr getPlane(const PointCloud::Ptr &full_cloud, const Eigen::Vector3f &axis,
                             pcl::ModelCoefficients::Ptr &coefficients,
                             pcl::PlanarPolygon<PointT>::Ptr &hull_polygon, PointCloud::Ptr &hull_pointcloud);
};

#endif
