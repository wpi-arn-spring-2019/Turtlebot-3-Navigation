#include <ros/ros.h>
#include <global_planner.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner_node");

    ros::NodeHandle nh;

    ros::NodeHandle pnh("~");

    Turtlebot::GlobalPlanner planner(nh, pnh);

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
