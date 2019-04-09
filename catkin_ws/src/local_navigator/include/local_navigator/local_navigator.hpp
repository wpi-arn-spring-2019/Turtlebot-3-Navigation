#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <point.hpp>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <turtlebot_msgs/GoalPose.h>

namespace Turtlebot
{

class LocalNavigator
{
public:
    LocalNavigator(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalNavigator(){}

    void setWaypoint();

private:
    void pathCallback(const nav_msgs::Path::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void localPlannerHealthCallback(const std_msgs::Bool::ConstPtr &msg);

    void initializeNavigator(ros::NodeHandle &pnh);
    void setupCollision();
    const bool checkForGoal();
    const turtlebot_msgs::GoalPose calcNextWaypoint();
    const tf::Pose calcGoalPose(const int &path_it_offset);
    const int calcNearestPathPoint();
    const double calcVelocityGoal(const tf::Pose &pose);
    const bool checkForCollision(const tf::Transform &transform) const;
    const bool checkPointForCollision(const tf::Point &pt) const;
    const tf::Transform calcPointTransform(const tf::Pose &pose) const;
    const int &calcGridLocation(const Point<double> &pt) const;
    void integrateOdomToCurrentTime();
    void integratePoseToCurrentTime();
    void publishGoalPose(const turtlebot_msgs::GoalPose &pose);
    void publishGoalStatus(const bool &reached);
    void pubReplan(const bool &replan);

    ros::Subscriber m_path_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_odom_sub;
    ros::Subscriber m_map_sub;\
    ros::Subscriber m_local_planner_health_sub;
    ros::Publisher m_goal_pose_pub;
    ros::Publisher m_goal_reached_pub;
    ros::Publisher m_replan_pub;

    nav_msgs::Path::ConstPtr m_path;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr m_pose;
    geometry_msgs::PoseWithCovarianceStamped m_pose_at_cur_time;
    nav_msgs::Odometry::ConstPtr m_odom;
    nav_msgs::Odometry m_odom_at_cur_time;
    nav_msgs::Odometry::ConstPtr m_prev_odom;
    nav_msgs::OccupancyGrid::ConstPtr m_map;

    ros::Time m_current_time;
    std::vector<tf::Point> m_collision_pts;

    double m_collision_buffer_distance;
    double m_waypoint_dist;
    double m_corridor_radius;
    double m_max_speed;
    double m_max_accel;
    double m_goal_pos_tol;
    double m_goal_ang_tol;
    double m_goal_vel_tol;
    double m_radius;
    double m_angular_col_res;
    double m_radial_col_res;

    bool m_have_path = false;
    bool m_have_pose = false;
    bool m_have_odom = false;
    bool m_have_map = false;

};

}
