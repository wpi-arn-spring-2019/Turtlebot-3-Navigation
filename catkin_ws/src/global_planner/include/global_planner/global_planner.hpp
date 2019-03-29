#pragma once
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <global_planner_types.hpp>
#include <unordered_map>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <turtlebot_msgs/GoalPose.h>
#include <queue>
#include <tf/transform_listener.h>
#include <urdf/model.h>

typedef std::vector<std::vector<double>> Graph;
typedef Eigen::Spline<double, 2> Spline1d;
typedef Eigen::SplineFitting<Spline1d> Spline1dFitting;

namespace Turtlebot
{

class GlobalPlanner
{

public:
    GlobalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~GlobalPlanner();

    void planPath();
    bool have_goal = false;

private:
    //callbacks
    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goalPoseCallback(const turtlebot_msgs::GoalPose::ConstPtr &msg);
    void rvizGoalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

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
    const tf::Transform calcNodeTransform(const Point<double> &pt) const;

    //cost functions
    const double calcG(const Point<double> &pt, const GraphNode &parent_node) const;
    const double calcH(const Point<double> &pt);
    const double calcDistanceBetween(const Point<double> &lhs, const Point<double> &rhs) const;

    //collision
    const bool checkForCollision(const tf::Transform &transform) const;
    const bool checkPointForCollision(const tf::Point &pt) const;

    //helper functions
    const int &calcGridLocation(const Point<double> &pt) const;

    //output functions
    void reconstructTrajectory();
    const std::vector<GraphNode> reverseTrajectory(const std::vector<GraphNode> &reverse_traj);
    const std::vector<GraphNode> interpolateTrajectory(const std::vector<GraphNode> &traj);
    void pubTrajectory(const std::vector<GraphNode> &traj);

    //pubsub
    ros::Subscriber m_costmap_sub;
    ros::Subscriber m_goal_pose_sub;
    ros::Subscriber m_rviz_goal_pose_sub;
    ros::Subscriber m_pose_sub;
    ros::Publisher m_path_pub;
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
    double m_global_costmap_res;
    int m_global_costmap_height;
    int m_global_costmap_width;
    double m_collision_buffer_distance;
    double m_search_res;
    double m_angular_search_res;
    double m_spline_order;
    double m_goal_pos_tolerance;
    int m_num_pts_final_path;
    double m_timeout_ms;
    double m_radius;
    double m_angular_col_res;
    double m_radial_col_res;

    //callback refs
    nav_msgs::OccupancyGrid::ConstPtr m_global_costmap;
    turtlebot_msgs::GoalPose m_goal_pose;
    nav_msgs::OccupancyGrid::ConstPtr m_map;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr m_pose;

    //flags    
    bool m_have_costmap = false;
    bool m_have_pose = false;
};

}
