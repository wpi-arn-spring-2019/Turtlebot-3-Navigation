#pragma once
#include <ros/ros.h>
#include <Eigen/Dense>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::registration::CorrespondenceEstimation<Point, Point> CorrespondenceEstimation;
typedef pcl::registration::TransformationEstimation2D<Point, Point> TransformationEstimation2D;

namespace Turtlebot
{

class PoseEstimationICP
{
public:
    PoseEstimationICP(){}
    ~PoseEstimationICP(){}

    const tf::Pose getTransform(const sensor_msgs::LaserScan &source_scan, const sensor_msgs::LaserScan &target_scan);

private:
    const PointCloud::Ptr convertToPCL(const sensor_msgs::LaserScan &scan);
    const Eigen::Matrix4f calcTransformICP(const PointCloud::Ptr &source_cloud, const PointCloud::Ptr &target_cloud);
    const tf::Pose convertMatrixToPose(const Eigen::Matrix4f &mat);

    laser_geometry::LaserProjection m_projector;
};

}
