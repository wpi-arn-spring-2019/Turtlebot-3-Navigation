#include <pose_estimation_icp.hpp>

namespace Turtlebot
{

const tf::Pose PoseEstimationICP::getTransform(const sensor_msgs::LaserScan &source_scan, const sensor_msgs::LaserScan &target_scan)
{
    const PointCloud::Ptr &source_cloud = convertToPCL(source_scan);
    const PointCloud::Ptr &target_cloud = convertToPCL(target_scan);
    return convertMatrixToPose(calcTransformICP(source_cloud, target_cloud));
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
    icp.setInputSource(target_cloud);
    icp.setInputTarget(source_cloud);
    PointCloud::Ptr out_cloud(new PointCloud);
    icp.align(*out_cloud);
    return icp.getFinalTransformation();
}

const tf::Pose PoseEstimationICP::convertMatrixToPose(const Eigen::Matrix4f &mat)
{
    tf::Pose pose;
    pose.getOrigin().setX(mat(0, 3));
    pose.getOrigin().setY(mat(1, 3));
    pose.getOrigin().setZ(mat(2, 3));
    tf::Matrix3x3 tf_mat;
    tf_mat.setValue(mat(0, 0), mat(0, 1), mat(0, 2),
                    mat(1, 0), mat(1, 1), mat(1, 2),
                    mat(2, 0), mat(2, 1), mat(2, 2));
    tf::Quaternion q;
    tf_mat.getRotation(q);
    q.normalize();
    pose.setRotation(q);
    return pose;
}

}
