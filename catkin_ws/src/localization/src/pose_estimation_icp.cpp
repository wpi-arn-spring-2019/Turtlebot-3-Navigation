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
    CorrespondenceEstimation::Ptr corr_est(new CorrespondenceEstimation);
    corr_est->setInputSource(source_cloud);
    corr_est->setInputTarget(target_cloud);
    pcl::Correspondences corr;
    corr_est->determineCorrespondences(corr);
    TransformationEstimation2D::Ptr trans_est(new TransformationEstimation2D);
    Eigen::Matrix4f est_mat;    
    trans_est->estimateRigidTransformation(*source_cloud, *target_cloud, corr, est_mat);
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setTransformationEstimation(trans_est);    
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    PointCloud out_cloud;
    icp.align(out_cloud);
    return icp.getFinalTransformation().inverse();
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
