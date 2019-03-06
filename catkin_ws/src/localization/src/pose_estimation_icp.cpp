#include <pose_estimation_icp.hpp>

namespace Turtlebot
{

const Eigen::Matrix4f PoseEstimationICP::getTransform(const sensor_msgs::LaserScan &source_scan, const sensor_msgs::LaserScan &target_scan)
{
    const PointCloud::Ptr &source_cloud = convertToPCL(source_scan);
    const PointCloud::Ptr &target_cloud = convertToPCL(target_scan);
    const Eigen::Matrix4f &transform = calcTransformICP(source_cloud, target_cloud);

}

const PointCloud::Ptr PoseEstimationICP::convertToPCL(const sensor_msgs::LaserScan &scan)
{
    sensor_msgs::PointCloud2 cloud;
    m_projector.projectLaser(scan, cloud);
    PointCloud::Ptr pcloud(new PointCloud);
    pcl::fromROSMsg(cloud, *pcloud);
    return pcloud;
}

const Eigen::Matrix4f PoseEstimationICP::calcTransformICP(const PointCloud::Ptr &source_cloud, const PointCloud::Ptr &target_cloud)
{
    WarpPointRigid3D::Ptr wpr3d(new WarpPointRigid3D);
    TransformationEstimationLM::Ptr trans_est(new TransformationEstimationLM);
    trans_est->setWarpFunction(wpr3d);
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setTransformationEstimation(trans_est);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    PointCloud::Ptr out_cloud(new PointCloud);
    icp.align(*out_cloud);
    return icp.getFinalTransformation();
}

}
