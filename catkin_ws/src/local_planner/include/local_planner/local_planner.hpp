#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <local_planner_types.hpp>
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <turtlebot_msgs/GoalPose.h>
#include <turtlebot_msgs/Trajectory.h>
#include <queue>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <urdf/model.h>

typedef std::vector<std::vector<double>> Graph;
typedef Eigen::Spline<double, 2> Spline1d;
typedef Eigen::SplineFitting<Spline1d> Spline1dFitting;

namespace Turtlebot
{

class LocalPlanner
{

public:
    LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~LocalPlanner();

    void planPath();
    bool have_goal = false;

private:
    //callbacks
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goalPoseCallback(const turtlebot_msgs::GoalPose::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    //setup methods
    void getParams(ros::NodeHandle &pnh);
    void setupCollision();

    //methods

    //expansion    
    void initializePlanner();
    void clearFrontier();
    void openNode(const GraphNode &node);
    void closeNode(const GraphNode &node);
    const bool checkForGoal(const GraphNode &node);
    void expandFrontier(const GraphNode &current_node);
    const std::vector<GraphNode> getNeighbors(const GraphNode &current_node);

    //sampling
    const std::vector<double> calcPossibleVelocities(const GraphNode &current_node) const;
    void calcPossibleYawRates();
    const tf::Transform calcNodeTransform(const Point<double> &pt, const double &heading) const;

    //cost functions
    const double calcG(const Point<double> &pt, const GraphNode &parent_node) const;
    const double calcH(const Point<double> &pt, const Point<double> &cur_pt, const double &heading, const double &velocity, const Spline1d &spline);
    const Spline1d calcSpline(const Point<double> &pt, const double &heading) const;
    const double calcTargetHeading(const Point<double> &pt, const double &velocity, const Spline1d &spline);
    const bool checkVelH(const Point<double> &pt, const double &heading, const double &velocity);
    const double calcDistanceBetween(const Point<double> &lhs, const Point<double> &rhs) const;

    //collision
    const bool checkForCollision(const tf::Transform &transform) const;
    const bool checkPointForCollision(const tf::Point &pt) const;

    //helper functions
    const int &calcGridLocation(const Point<double> &pt) const;

    //output functions
    void reconstructTrajectory();
    const std::vector<GraphNode> reverseTrajectory(const std::vector<GraphNode> &reverse_traj);
    void pubTrajectory(const std::vector<GraphNode> &traj);

    //pubsub
    ros::Subscriber m_costmap_sub;
    ros::Subscriber m_goal_pose_sub;
    ros::Subscriber m_pose_sub;
    ros::Subscriber m_odom_sub;
    ros::Publisher m_path_pub;
    ros::Publisher m_trajectory_pub;
    ros::Publisher m_goal_pub;

    //members

    //frontier and nodes
    std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost> m_frontier;
    std::vector<GraphNode> m_open_nodes;
    std::vector<GraphNode> m_closed_nodes;
    std::unordered_map<int, GraphNode> m_nodes;
    int m_node_id = 0;

    //collision
    std::vector<tf::Point> m_collision_pts;

    //tf listener
    const tf::TransformListener listener;


    //c-space stuff and constricting parameters
    std::vector<double> m_possible_yaw_rates;
    double m_local_costmap_res;
    int m_local_costmap_height;
    int m_local_costmap_width;
    double m_collision_buffer_distance;
    double m_time_step_ms;
    double m_velocity_res;
    double m_yaw_rate_res;
    double m_max_yaw_rate;
    double m_spline_order;
    double m_goal_pos_tolerance;
    double m_goal_heading_tolerance;
    double m_goal_speed_tolerance;
    double m_timeout_ms;
    int m_num_retries;
    double m_radius;
    double m_angular_col_res;
    double m_radial_col_res;

    //callback refs
    nav_msgs::OccupancyGrid::ConstPtr m_local_costmap;
    turtlebot_msgs::GoalPose m_goal_pose;
    nav_msgs::OccupancyGrid::ConstPtr m_map;
    geometry_msgs::Twist m_turtlebot_velocity;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr m_pose;

    //flags    
    bool m_have_costmap = false;
    bool m_have_pose = false;
    bool m_have_odom = false;
    int m_retry_it = 0;
};

}
