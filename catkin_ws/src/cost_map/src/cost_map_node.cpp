#include <ros/ros.h>
#include <cost_map.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cost_map_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
    ROS_INFO("cost_map_node initialization start");

  Turtlebot::CostMap costmap(nh, pnh);

  while(ros::ok)
  {
      ros::spin();
  }

  return 0;
}



