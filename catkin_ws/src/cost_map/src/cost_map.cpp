#include <ros/ros.h>
#include <cost_map.hpp>




namespace Turtlebot
{

CostMap::CostMap(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&CostMap::mapCallback,this);
    current_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &CostMap::currentScanCallback, this);
    current_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose",10,&CostMap::currentPoseCallback, this);
    cost_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map",10);
}

CostMap::~CostMap(){}

void CostMap::currentScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    if(!m_have_scan && m_have_odom && m_have_map)
    {
        m_have_scan = true;
        m_prev_scan = *msg;
        m_fake_scan = new FakeScan(*m_map, *msg);
        m_odom_at_last_scan = *m_odom;
     }
     current_scan = *msg;
     scan_available = 1;
     if(m_have_odom)
     {
         integrateOdomToScanTime();
     }

}
void CostMap::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_prev_odom = msg;
    }
    m_odom = msg;
}
void CostMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
   if(!m_have_map)
   {
    m_have_map = true;
    m_map = msg;
    cost_map = *msg;
 }
}
void CostMap::currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose_estimate)
    {
        m_have_pose_estimate = true;
    }
   current_pose = *msg;

   tf::Quaternion q;
   q.setW(current_pose.pose.pose.orientation.w);
   q.setX(current_pose.pose.pose.orientation.x);
   q.setY(current_pose.pose.pose.orientation.y);
   q.setZ(current_pose.pose.pose.orientation.z);
   double actual_roll, actual_pitch, actual_yaw;
   tf::Matrix3x3(q).getRPY(actual_roll, actual_pitch, actual_yaw);

   tf::Pose current_tf_pose;
   current_tf_pose.setOrigin(tf::Vector3(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, 0));
   current_tf_pose.setRotation(tf::createQuaternionFromYaw(actual_yaw));

   integratePoseToCurrentTime(current_tf_pose);

  if(m_have_map && m_have_scan)
  {
  referance_scan = m_fake_scan->getFakeScan(m_prev_pose);
//  referance_scan = fake_scan.getFakeScan(current_pose);
  generateCostMap();
  }
}


void CostMap::integratePoseToCurrentTime(tf::Pose &pose)
{
    ros::Duration dt = ros::Time::now() - m_odom_at_scan.header.stamp;
    const double &acc_x = (m_odom_at_scan.twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom_at_scan.twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom_at_scan.twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    m_odom_at_scan.twist.twist.linear.x = m_odom_at_scan.twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_scan.twist.twist.linear.y = m_odom_at_scan.twist.twist.linear.y + acc_y * dt.toSec();
    m_odom_at_scan.twist.twist.angular.z = m_odom_at_scan.twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_at_scan.pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &radius_curvature = std::pow(m_odom_at_scan.twist.twist.linear.x, 2) +
                                     std::pow(m_odom_at_scan.twist.twist.linear.y, 2) /
                                     m_odom_at_scan.twist.twist.angular.z;
    const double &yaw_f = yaw + m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    double x_f;
    double y_f;
    if(std::isnan(radius_curvature))
    {
        x_f = m_odom_at_scan.pose.pose.position.x + (m_odom_at_scan.twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2) * cos(yaw);
        y_f = m_odom_at_scan.pose.pose.position.y + (m_odom_at_scan.twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2) * sin(yaw);
    }
    else
    {
        x_f = m_odom_at_scan.pose.pose.position.x + radius_curvature * cos(yaw_f) * cos(yaw);
        y_f = m_odom_at_scan.pose.pose.position.y + radius_curvature * sin(yaw_f) * sin(yaw);
    }

    current_pose.pose.pose.position.x = x_f;
    current_pose.pose.pose.position.y = y_f;
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    current_pose.pose.pose.orientation.w = q_f.w();
    current_pose.pose.pose.orientation.x = q_f.x();
    current_pose.pose.pose.orientation.y = q_f.y();
    current_pose.pose.pose.orientation.z = q_f.z();

//    pose.setOrigin(tf::Vector3(x_f, y_f, 0));
//    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
//    pose.setRotation(q_f);
}




void CostMap::generateCostMap()
{
 for (int i ; i<= current_scan.ranges.size(); i++)
  {
      if (std::abs(referance_scan.ranges[i] - current_scan.ranges[i]) > threshold)
      {
          tf::Quaternion q;
          q.setW(current_pose.pose.pose.orientation.w);
          q.setX(current_pose.pose.pose.orientation.x);
          q.setY(current_pose.pose.pose.orientation.y);
          q.setZ(current_pose.pose.pose.orientation.z);
          double roll, pitch, yaw;
          tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

             int x =  current_scan.ranges[i]  *  cos( yaw + (i*current_scan.angle_increment));
             int y = current_scan.ranges[i] * sin( yaw + (i*current_scan.angle_increment));


             cost_map.data[(y*cost_map.info.width + x)+1] = 100;
             cost_map_pub.publish(cost_map);
      }

  }

}




void CostMap::integrateOdomToScanTime()
{
    ros::Duration dt = m_scan->header.stamp - m_odom->header.stamp;
    m_odom_at_scan.header.stamp = m_scan->header.stamp;
    const double &acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    m_odom_at_scan.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_scan.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt.toSec();
    m_odom_at_scan.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    const double &x_f = m_odom->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2) * cos(yaw);
    const double &y_f = m_odom->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_odom_at_scan.pose.pose.position.x = x_f;
    m_odom_at_scan.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_scan.pose.pose.orientation = q_;
}



}
