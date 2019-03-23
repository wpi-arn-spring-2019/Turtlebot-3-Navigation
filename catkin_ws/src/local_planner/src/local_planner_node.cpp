#include <ros/ros.h>
#include <local_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::LocalPlanner planner(nh, pnh);

    ros::Rate r(10);

    while(ros::ok)
    {
        ros::spinOnce();

        if(planner.have_goal)
        {
            planner.planPath();
        }

        r.sleep();
    }

    return 0;

}
