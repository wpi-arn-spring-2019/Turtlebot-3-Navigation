#include <ros/ros.h>
#include <cost_map.hpp>


namespace Turtlebot
{

CostMap::CostMap(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    ROS_INFO("cost_map_node initiated");
    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map",10,&CostMap::mapCallback,this);
    ROS_INFO("map has been subscribed");
    current_scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &CostMap::currentScanCallback, this);
    ROS_INFO("scan has been subscribed");
    ROS_INFO("commit");

    cost_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map",10);
    ROS_INFO("Able to publish cost map to the topic cost_map now");
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &CostMap::odomCallback, this);

    current_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose",10,&CostMap::currentPoseCallback, this);


    ROS_INFO("pf pose has been subscribed");

}

CostMap::~CostMap(){}

void CostMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{

   if(!m_have_map)
   {
    m_have_map = true;
    m_map = msg;
    cost_map = *msg;
    ROS_INFO("CALLBACK map");

//    for (int i=0; i<cost_map.data.size(); i++)
//    {
//      if(cost_map.data[i]==100)
//      {
//      ROS_INFO("%d position %d value",i,cost_map.data[i]);
//      ROS_INFO("m_map and cost_map is taken");
//      }

//    }

   }
}

void CostMap::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

    if(!m_have_odom)
    {
        m_have_odom = true;
        m_prev_odom = msg;
        ROS_INFO("CALLBACK odom");
    }
    m_odom = msg;
}


void CostMap::currentScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    if(!m_have_scan && m_have_odom && m_have_map)
    {
        m_have_scan = true;
        m_prev_scan = *msg;
        ROS_INFO("CALLBACK scan");

        m_fake_scan = new FakeScan(*m_map, *msg);
//        ROS_INFO("first scan is gone to m_prev scan");
        m_odom_at_last_scan = *m_odom;
    }
     m_scan = msg;
    if(m_have_odom)
    {
        integrateOdomToScanTime();
    }

}

void CostMap::currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose_estimate)
    {
        m_have_pose_estimate = true;
    }
    current_pose = *msg;
    current_cartesian_x = current_pose.pose.pose.position.x;
    current_cartesian_y = current_pose.pose.pose.position.y;
    current_grid_x = int(current_pose.pose.pose.position.x/0.05);
    current_grid_y = int(current_pose.pose.pose.position.y/0.05);
    map_index_num = ((current_grid_x+200) *cost_map.info.width) +(current_grid_y+200);
//   tf::Quaternion q;
//   q.setW(current_pose.pose.pose.orientation.w);
//   q.setX(current_pose.pose.pose.orientation.x);
//   q.setY(current_pose.pose.pose.orientation.y);
//   q.setZ(current_pose.pose.pose.orientation.z);
//   double actual_roll, actual_pitch, actual_yaw;
//   tf::Matrix3x3(q).getRPY(actual_roll, actual_pitch, actual_yaw);
//   tf::Pose current_tf_pose;
//   current_tf_pose.setOrigin(tf::Vector3(current_pose.pose.pose.position.x, current_pose.pose.pose.position.y, 0));
//   current_tf_pose.setRotation(tf::createQuaternionFromYaw(actual_yaw));

//   ROS_INFO("current pose before 1 %d %d ",current_pose.pose.pose.position.x,current_pose.pose.pose.position.y);
//   integratePoseToCurrentTime(current_tf_pose);

     referance_scan = m_fake_scan->getFakeScan(current_pose);
//  for (int i=0 ; i<= referance_scan.ranges.size(); i++)
//  {
//    ROS_INFO("ranges position %f",referance_scan.ranges[i]);
//  }

    ROS_INFO("y x width index_num %d %d %d %d",current_grid_x,current_grid_y,cost_map.info.width, map_index_num);
//   cost_map.data[int((192+8)*cost_map.info.width + (152+8))] = 100;
//      cost_map.data[int((152)*cost_map.info.width + (192))] = 100;
//      cost_map.data[int((152)*cost_map.info.width + (192))] = 100;
//      cost_map.data[int((152)*cost_map.info.width + (192))] = 100;
//   cost_map_pub.publish(cost_map);

    ROS_INFO("cost map value %d",cost_map.data[map_index_num]);
    if(cost_map.data[map_index_num]!= -1)
    {
      ROS_INFO("cost map pixel value %d",cost_map.data[map_index_num]);
      generateCostMap();
    }

}

void CostMap::generateCostMap()
{
 ROS_INFO("new current pose 4444444444444 ");
    for (int i=0 ; i<= m_scan->ranges.size(); i++)
    {

//      clearCostMap();
        ///////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////
        //modify this quaternion

        tf::Quaternion q;
        q.setW(current_pose.pose.pose.orientation.w);
        q.setX(current_pose.pose.pose.orientation.x);
        q.setY(current_pose.pose.pose.orientation.y);
        q.setZ(current_pose.pose.pose.orientation.z);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double ray_dist = 0;   // find the minimum ray_dist and put it instead of zero

        /////////////////////////////////////////////////////////////////////////////////////////

        double angle = yaw + (i*m_scan->angle_increment);

//        const double &x = (current_grid_x+200) + ray_dist * cos(angle);
//        const double &y = (current_grid_y+200) + ray_dist * sin(angle);

        ROS_INFO("start throwing laser range %f",m_scan->ranges[i]);

        while(ray_dist <= m_scan->ranges[i] && ray_dist <= m_scan->range_max)
        {

            int pix_x = (current_grid_x+200) + int((ray_dist * cos(angle))/0.05);
            int pix_y = (current_grid_y+200) + int((ray_dist * sin(angle))/0.05);
//            ROS_INFO("pix_x pix_y costMapValue ray Distance %d %d %f %f",pix_x,pix_y,cost_map.data[(pix_y*cost_map.info.width + pix_x)],ray_dist);
            cost_map.data[(pix_y*cost_map.info.width + pix_x)] = 0;
            ray_dist += 0.001;
            cost_map_pub.publish(cost_map);
        }

   ////////////////////////////////////////////
        if(!(m_scan->ranges[i]>m_scan->range_max) && !(m_scan->ranges[i]<m_scan->range_min))
        {
      if ((referance_scan.ranges[i] - m_scan->ranges[i]) > 0.5)
      {
//            tf::Quaternion q;
//            q.setW(current_pose.pose.pose.orientation.w);
//            q.setX(current_pose.pose.pose.orientation.x);
//            q.setY(current_pose.pose.pose.orientation.y);
//            q.setZ(current_pose.pose.pose.orientation.z);
//            double roll, pitch, yaw;

//            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
             angle = yaw + (i*m_scan->angle_increment);


//            ROS_INFO("current grid x %d add %d",current_grid_x+200,  int((m_scan->ranges[i] * cos(angle))/0.05));
//            ROS_INFO("current grid y %d add %d",current_grid_y+200,  int((m_scan->ranges[i] * sin(angle))/0.05));

            int x = (current_grid_x+200) + int((m_scan->ranges[i] * cos(angle))/0.05);
            int y = (current_grid_y+200) + int((m_scan->ranges[i] * sin(angle))/0.05);


            cost_map.data[(y*cost_map.info.width + x)] = 100;


             cost_map_pub.publish(cost_map);
        }
      }

  }


}

void CostMap::clearCostMap()
{



}


void CostMap::integrateOdomToScanTime()
{
    ROS_INFO("integrate odom to scan 0 ");

//    ROS_INFO("integrate odom to scan 0 %d %d ",m_scan->header.stamp,m_odom->header.stamp);
    ros::Duration dt = m_scan->header.stamp - m_odom->header.stamp;
//    ROS_INFO("integrate odom to scan1");
    m_odom_at_scan.header.stamp = m_scan->header.stamp;
//    ROS_INFO("integrate odom to scan 2");
    const double &acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
//    ROS_INFO("integrate odom to scan 3");
    const double &acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
//    ROS_INFO("integrate odom to scan 4");
    const double &acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
//    ROS_INFO("integrate odom to scan 5");
    m_odom_at_scan.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt.toSec();
//    ROS_INFO("integrate odom to scan 6");
    m_odom_at_scan.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt.toSec();
//    ROS_INFO("integrate odom to scan 7");
    m_odom_at_scan.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_scan.twist.twist.linear.z * dt.toSec();
    const double &x_f = m_odom->pose.pose.position.x + (m_odom->twist.twist.linear.x * dt.toSec() + acc_x * std::pow(dt.toSec(), 2) / 2) * cos(yaw);
    const double &y_f = m_odom->pose.pose.position.y + (m_odom->twist.twist.linear.y * dt.toSec() + acc_y * std::pow(dt.toSec(), 2) / 2) * sin(yaw);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
//    ROS_INFO("integrate odom to scan 8");
    q_f.normalize();
    m_odom_at_scan.pose.pose.position.x = x_f;
    m_odom_at_scan.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_scan.pose.pose.orientation = q_;

    m_prev_scan = *m_scan;
    m_prev_odom = m_odom;
//    ROS_INFO("integrate odom to scan 9");


}













void CostMap::integratePoseToCurrentTime(tf::Pose &pose)
{
    ROS_INFO("INTEGRATE POSE TO TIME 0");
    ros::Duration dt = ros::Time::now() - m_odom_at_scan.header.stamp;
//    ROS_INFO("INTEGRATE POSE TO TIME 1");
    const double &acc_x = (m_odom_at_scan.twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom_at_scan.twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom_at_scan.twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
//    ROS_INFO("INTEGRATE POSE TO TIME 2");
    m_odom_at_scan.twist.twist.linear.x = m_odom_at_scan.twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_scan.twist.twist.linear.y = m_odom_at_scan.twist.twist.linear.y + acc_y * dt.toSec();
//    ROS_INFO("INTEGRATE POSE TO TIME 3");
    m_odom_at_scan.twist.twist.angular.z = m_odom_at_scan.twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom_at_scan.pose.pose.orientation, q);
//     ROS_INFO("INTEGRATE POSE TO TIME 4");
    const double &yaw = tf::getYaw(q);
//    ROS_INFO("INTEGRATE POSE TO TIME 4 5 ");
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
//        ROS_INFO("INTEGRATE POSE TO TIME 5");
    }
    else
    {
        x_f = m_odom_at_scan.pose.pose.position.x + radius_curvature * cos(yaw_f) * cos(yaw);
        y_f = m_odom_at_scan.pose.pose.position.y + radius_curvature * sin(yaw_f) * sin(yaw);
//        ROS_INFO("INTEGRATE POSE TO TIME 6");
    }

//    current_pose.pose.pose.position.x = x_f;
//    current_pose.pose.pose.position.y = y_f;
////    ROS_INFO("INTEGRATE POSE TO TIME 7");
//    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
//    ROS_INFO("INTEGRATE POSE TO TIME 8");
//    current_pose.pose.pose.orientation.w = q_f.w();
//    current_pose.pose.pose.orientation.x = q_f.x();
//    current_pose.pose.pose.orientation.y = q_f.y();
//    current_pose.pose.pose.orientation.z = q_f.z();

//    pose.setOrigin(tf::Vector3(x_f, y_f, 0));
//    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
//    pose.setRotation(q_f);
}



}
