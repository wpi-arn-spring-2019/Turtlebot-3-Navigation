#include <local_planner.hpp>

namespace Turtlebot
{

LocalPlanner::LocalPlanner(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
    m_costmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/local_costmap", 10, &LocalPlanner::costmapCallback, this);
    m_goal_pose_sub = nh.subscribe<turtlebot_msgs::GoalPose>("/goal_pose", 1, &LocalPlanner::goalPoseCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/pf_pose", 10, &LocalPlanner::poseCallback, this);
    m_trajectory_pub = nh.advertise<turtlebot_msgs::Trajectory>("/trajectory", 10);
    m_occ_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/planning_scene", 1);
    m_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
    m_spline_pub = nh.advertise<nav_msgs::Path>("/spline", 100);
    getParams(pnh);
    setupCollision();
    setupCSpace();
}

LocalPlanner::~LocalPlanner(){}

void LocalPlanner::getParams(ros::NodeHandle &pnh)
{
      double costmap_height_meters;
      double costmap_width_meters;
      pnh.getParam("/local_costmap_node/local_costmap_res", m_local_costmap_res);
      pnh.getParam("/local_costmap_node/local_costmap_height", costmap_height_meters);
      pnh.getParam("/local_costmap_node/local_costmap_width", costmap_width_meters);
      pnh.getParam("collision_buffer_distance", m_collision_buffer_distance);
      pnh.getParam("base_time_step_ms", m_base_time_step_ms);
      pnh.getParam("velocity_res", m_velocity_res);
      pnh.getParam("heading_res", m_heading_res);
      pnh.getParam("max_yaw_rate", m_max_yaw_rate);
      pnh.getParam("spline_order", m_spline_order);
      pnh.getParam("goal_pos_tolerance", m_goal_pos_tolerance);
      pnh.getParam("goal_heading_tolerance", m_goal_heading_tolerance);
      pnh.getParam("goal_speed_tolerance", m_goal_speed_tolerance);
      pnh.getParam("timeout_ms", m_timeout_ms);
      m_local_costmap_height = costmap_height_meters / m_local_costmap_res;
      m_local_costmap_width = costmap_width_meters / m_local_costmap_res;
}

void LocalPlanner::setupCollision()
{
      tf::StampedTransform transform;
      getTransform("front_right_middle_sonar_link", "back_right_middle_sonar_link", transform);
      const double &length = fabs(transform.getOrigin().getX()) + 2 * m_collision_buffer_distance;
      getTransform("rear_right_wheel", "rear_left_wheel", transform);
      const double &width = fabs(transform.getOrigin().getX()) + 2 * m_collision_buffer_distance;
      getTransform("back_right_middle_sonar_link", "base_link", transform);
      const double &center_offset = fabs(transform.getOrigin().getY());
      const int &num_pts = length * width / m_local_costmap_res;
      int row = 0;
      int col = 0;
      for(int i = 0; i < num_pts; i++)
      {
          tf::Point pt;
          pt.setX(col * m_local_costmap_res);
          pt.setY(row * m_local_costmap_res - center_offset);
          pt.setZ(0);
          m_collision_pts.push_back(pt);
          if(col > int(width / m_local_costmap_res))
          {
              row++;
              col = 0;
          }
          else
          {
              col++;
          }
      }
}

void LocalPlanner::setupCSpace()
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space[i].push_back(0);
        }
    }
}

void LocalPlanner::planPath()
{
    initializePlanner();
    const ros::Time &start_time = ros::Time::now();
    ros::Duration duration;
    int count = 0;
    while(true)
    {
        duration = ros::Time::now() - start_time;
        if(duration.toSec() * 1000 > m_timeout_ms)
        {
            ROS_ERROR_STREAM("path plan time exceeded, attempting to replan");
            return;
        }
        const GraphNode &current_node = m_frontier.top();
        m_frontier.pop();
        std::pair<bool, GraphNode> goal_result = checkForGoal(current_node);
        if(goal_result.first)
        {
            ROS_INFO_STREAM("Path found in " << duration.toSec() << " seconds");
            break;
        }
        closeNode(current_node);
        expandFrontier(current_node);
        if(count == 1)
        {
            count = 0;
            //calcOccGrid();
        }
        count++;
    }        
}

void LocalPlanner::getTransform(const std::string &link1, const std::string &link2, tf::StampedTransform &transform) const
{
    while(!listener.canTransform(link1, link2, ros::Time(0)))
    {
        ros::Duration(0.01).sleep();
    }
    try
    {
        listener.lookupTransform(link1, link2, ros::Time(0), transform);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

void LocalPlanner::initializePlanner()
{
    clearVisited();
    clearFrontier();    
}

void LocalPlanner::clearFrontier()
{
    m_frontier = std::priority_queue<GraphNode, std::vector<GraphNode>, GraphNode::CheaperCost>();
    m_open_nodes.clear();
    m_closed_nodes.clear();
    double roll, pitch, yaw;
    tf::Quaternion q;
    q.setW(m_pose->pose.orientation.w);
    q.setX(m_pose->pose.orientation.x);
    q.setY(m_pose->pose.orientation.y);
    q.setZ(m_pose->pose.orientation.z);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    const double &vel = std::sqrt(std::pow(m_prius_velocity.linear.x, 2) + std::pow(m_prius_velocity.linear.y, 2));
    const GraphNode &node = GraphNode(Point<double>(0.0, 0.0), Point<double>(0.0, 0.0), yaw, vel, 0.0, std::numeric_limits<double>::infinity());
    m_frontier.push(node);
    openNode(node);
}

void LocalPlanner::openNode(const GraphNode &node)
{
    m_frontier.push(node);
    m_open_nodes.push_back(node);
}

void LocalPlanner::closeNode(const GraphNode &node)
{
    const std::vector<GraphNode>::iterator &it = std::find(m_open_nodes.begin(), m_open_nodes.end(), node);
    if(it != m_open_nodes.end())
    {
        m_closed_nodes.push_back(*it);
    }
    else
    {
        return;
    }
    m_open_nodes.erase(it);
}

const std::pair<bool, GraphNode> LocalPlanner::checkForGoal(const GraphNode &node)
{
    const double &time_step = m_time_step_ms / 1000;
    const double &dist_to_goal = calcDistanceBetween(node.child_point, Point<double>(m_goal_pose->x, m_goal_pose->y));
    const double &max_travel_dist = node.velocity * time_step + m_goal_pos_tolerance;
    const double &heading_diff = fabs(m_goal_pose->heading - node.heading);
    const double &velocity_diff = fabs(m_goal_pose->speed - node.velocity);
    if((dist_to_goal > max_travel_dist + m_goal_pos_tolerance) ||
       (heading_diff > m_max_yaw_rate * time_step + m_goal_heading_tolerance) ||
       (velocity_diff > m_goal_pose->max_accel * time_step + m_goal_speed_tolerance))
    {
        const GraphNode dummy_node(Point<double>(0, 0), Point<double>(0, 0), 0, 0, 0, 0);
        return std::make_pair(false, dummy_node);
    }
    const GraphNode goal_node(Point<double>(m_goal_pose->x, m_goal_pose->y), node.child_point, m_goal_pose->heading, m_goal_pose->speed, 0, 0);
    return std::make_pair(true, goal_node);
}

void LocalPlanner::expandFrontier(const GraphNode &current_node)
{
    const std::vector<GraphNode> &neighbors = getNeighbors(current_node);
    for(const auto &neighbor : neighbors)
    {
        markVisited(neighbor.child_point);
        const std::vector<GraphNode>::iterator &it_closed = std::find(m_closed_nodes.begin(), m_closed_nodes.end(), neighbor);
        if(it_closed != m_closed_nodes.end())
        {
           continue;
        }
        const std::vector<GraphNode>::iterator &it_open = std::find(m_open_nodes.begin(), m_open_nodes.end(), neighbor);
        if(it_open != m_open_nodes.end())
        {
            if(neighbor.cost >= (*it_open).cost)
            {
                continue;
            }
        }
        else
        {
            openNode(neighbor);
        }
    }
}

const std::vector<GraphNode> LocalPlanner::getNeighbors(const GraphNode &current_node)
{
    std::vector<GraphNode> neighbors = {};
    const Spline1d &spline = calcSpline(current_node.child_point, current_node.heading);
    pubSpline(spline);
    calcTimeStep(current_node.velocity);
    const std::vector<double> &possible_velocities = calcPossibleVelocities(current_node);
    const std::vector<double> &possible_headings = calcPossibleHeadings(current_node);    
    for(const auto &velocity : possible_velocities)
    {
        for(const auto &heading : possible_headings)
        {
            const double &average_velocity = (velocity + current_node.velocity) / 2;
            const double &x = current_node.child_point.x + average_velocity * m_time_step_ms / 1000 * cos(heading);
            const double &y = current_node.child_point.y + average_velocity * m_time_step_ms / 1000 * sin(heading);
            const Point<double> new_pt(x, y);
            if(!checkForCollision(calcNeighborTransform(new_pt, heading)))
            {
                const double &g = calcG(new_pt, current_node);
                const double &h = calcH(new_pt, current_node.child_point, heading, velocity, spline);
                neighbors.push_back(GraphNode(new_pt, current_node.child_point, heading, velocity, g, g + h));
            }
        }
    }    
    return neighbors;
}

const std::vector<double> LocalPlanner::calcPossibleVelocities(const GraphNode &current_node) const
{
    std::vector<double> possible_velocities = {};
    const double &current_velocity = current_node.velocity;
    const double &max_accel = m_goal_pose->max_accel;
    const double &max_velocity  = m_goal_pose->max_speed;
    const double &time_step = m_time_step_ms / 1000;
    const double &max_possible_vel = current_velocity + max_accel * time_step;
    const double &min_possible_vel = current_velocity - max_accel * time_step;
    const double &possible_vel_range = max_possible_vel - min_possible_vel;
    const int &num_possible_vels = possible_vel_range / m_velocity_res;
    for(int i = 0; i < num_possible_vels; i++)
    {
        const double &velocity = min_possible_vel + i * m_velocity_res;
        if(velocity == 0 && m_goal_pose->speed != 0)
        {
            continue;
        }
        if(velocity > max_velocity || velocity < 0)
        {
            continue;
        }
        possible_velocities.push_back(velocity);
    }
    return possible_velocities;
}

const std::vector<double> LocalPlanner::calcPossibleHeadings(const GraphNode &current_node) const
{
    std::vector<double> possible_yaws = {};
    const double &time_step = m_time_step_ms / 1000;
    const double &current_heading = current_node.heading;
    const double &max_heading = current_heading + m_max_yaw_rate * time_step;
    const double &min_heading = current_heading - m_max_yaw_rate * time_step;
    const double &heading_range = max_heading - min_heading;
    const double &num_possible_headings = heading_range / m_heading_res;
    for(int i = 0; i < num_possible_headings; i++)
    {
        const double &heading = min_heading + i * m_heading_res;
        possible_yaws.push_back(heading);
    }
    return possible_yaws;
}

const tf::Transform LocalPlanner::calcNeighborTransform(const Point<double> &pt, const double &heading) const
{
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pt.x, pt.y, 0));
    transform.setRotation(tf::createQuaternionFromYaw(heading));
    return transform;
}

void LocalPlanner::calcTimeStep(double velocity)
{
    if(velocity < 1)
    {
        velocity = 1;
    }
    m_time_step_ms = m_base_time_step_ms / (velocity);
}

const double LocalPlanner::calcG(const Point<double> &pt, const GraphNode &parent_node) const
{
    return parent_node.g + calcDistanceBetween(pt, parent_node.child_point);
}

const double LocalPlanner::calcH(const Point<double> &pt, const Point<double> &cur_pt, const double &heading, const double &velocity, const Spline1d &spline)
{
    const double &goal_x = m_goal_pose->x;
    const double &goal_y = m_goal_pose->y;
    const double &max_dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));
    const double &dist_to_goal = calcDistanceBetween(pt, Point<double>(m_goal_pose->x, m_goal_pose->y));
    const double &dist_heuristic = dist_to_goal / max_dist * 100;
    const double &target_heading = calcTargetHeading(cur_pt, velocity, spline);
    const double &heading_diff = fabs(target_heading - heading);
    double heading_heuristic = heading_diff / M_PI * 250;
    const double &costmap_value = abs(getCSpaceValue(cartesianToCSpace(pt)));
    double velocity_heuristic = 100 - velocity / m_goal_pose->max_speed * 100;
    if(checkVelH(pt, heading, velocity))
    {
        velocity_heuristic = fabs(velocity - m_goal_pose->speed) / (m_goal_pose->speed - m_goal_pose->max_speed) * 100;
    }
    return fabs(dist_heuristic + heading_heuristic + velocity_heuristic);
}

const Spline1d LocalPlanner::calcSpline(const Point<double> &pt, const double &heading) const
{
    const double &start_x = pt.x;
    const double &start_y = pt.y;
    const double &start_x_ = start_x + 0.001 * cos(heading);
    const double &start_y_ = start_y + 0.001 * sin(heading);
    const double &end_x = m_goal_pose->x;
    const double &end_y = m_goal_pose->y;
    const double &end_x_ = end_x - 0.001 * cos(m_goal_pose->heading);
    const double &end_y_ = end_y - 0.001 * sin(m_goal_pose->heading);
    Eigen::MatrixXd points(2, 4);
    points << start_x, start_x_, end_x_, end_x,
              start_y, start_y_, end_y_, end_y;
    const Spline1d &spline = Spline1dFitting::Interpolate(points, m_spline_order);
    return spline;
}

const double LocalPlanner::calcTargetHeading(const Point<double> &pt, const double &velocity, const Spline1d &spline)
{
    const double &dx = pt.x + velocity * m_time_step_ms / 1000;
    const double &dy = pt.y + velocity * m_time_step_ms / 1000;
    const double &dist = sqrt(pow(dx, 2) + pow(dy, 2));
    const double &spline_res = 0.01;
    double spline_value = 0;
    double spline_dist = 0;
    double x, y;
    double prev_x = pt.x;
    double prev_y  = pt.y;
    while(spline_dist < dist)
    {
        const Eigen::MatrixXd &spline_pt = spline(spline_value);
        x = spline_pt(0);
        y = spline_pt(1);
        const double &dp = sqrt(pow(x - prev_x, 2) + pow(y - prev_y, 2));
        spline_dist += dp;
        spline_value += spline_res;
    }
    return atan2(y - pt.y, x - pt.x);
}

const bool LocalPlanner::checkVelH(const Point<double> &pt, const double &heading, const double &velocity)
{
    double next_vel = velocity + m_goal_pose->max_accel * m_time_step_ms / 1000;
    if(next_vel > m_goal_pose->max_speed)
    {
        next_vel = m_goal_pose->max_speed;
    }
    const double &average_vel = (next_vel + velocity) / 2;
    const double &x = pt.x + average_vel * m_time_step_ms / 1000 * cos(heading);
    const double &y = pt.y + average_vel * m_time_step_ms / 1000 * sin(heading);
    const double &dx = x - m_goal_pose->x;
    const double &dy = y - m_goal_pose->y;
    const double &dist_to_goal = sqrt(pow(dx, 2) + pow(dy, 2));
    const double &speed = velocity;
    const double &max_accel = m_goal_pose->max_accel;
    const double &time_to_decel = (speed - m_goal_pose->speed) / max_accel;
    const double &dist_to_decel = speed * time_to_decel - m_goal_pose->max_accel * pow(time_to_decel, 2) / 2;
    return (dist_to_goal <= dist_to_decel);
}

const double LocalPlanner::calcDistanceBetween(const Point<double> &lhs, const Point<double> &rhs) const
{
    const double &dx = lhs.x - rhs.x;
    const double &dy = lhs.y - rhs.y;
    return std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
}

const bool LocalPlanner::checkForCollision(const tf::Transform &transform) const
{
    return false;
    for(const auto &pt : m_collision_pts)
    {
        const tf::Point &t_pt = transform * pt;

        if(checkPointForCollision(t_pt))
        {
           return true;
        }
    }
    return false;
}

const bool LocalPlanner::checkPointForCollision(const tf::Point &pt) const
{
    if(m_c_space[int(pt.x() / m_local_costmap_res)][int(pt.y() / m_local_costmap_res)] == 100)
    {
        return true;
    }
    return false;
}

void LocalPlanner::mapCostmapToCSpace(const nav_msgs::OccupancyGrid::ConstPtr &local_costmap)
{
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            const int &location = calcGridLocation(Point<int>(i, j));
            m_c_space[i][j] = local_costmap->data[location];
        }
    }
}

const int &LocalPlanner::calcGridLocation(const Point<int> &pt) const
{
    const int &location = m_local_costmap_height * m_local_costmap_width - pt.x * m_local_costmap_height - pt.y - 1;
    return location;
}

const int LocalPlanner::getCSpaceValue(const Point<int> &pt) const
{
    return m_c_space[pt.x][pt.y];
}

const Point<int> LocalPlanner::cartesianToCSpace(const Point<double> &pt)
{
    const int &x = m_c_space.size() / 2 + pt.x / m_local_costmap_res - 1;
    const int &y = m_c_space.size() / 2 - pt.y / m_local_costmap_res - 1;
    return Point<int>(x, y);
}

void LocalPlanner::clearVisited()
{
    m_c_space_visited.clear();
    for(int i = 0; i < m_local_costmap_width; i++)
    {
        m_c_space_visited.push_back({});
        for(int j = 0; j < m_local_costmap_height; j++)
        {
            m_c_space_visited[i].push_back(0);
        }
    }
}

void LocalPlanner::markVisited(const Point<double> &pt)
{
    const int &x = m_local_costmap_width / 2 - pt.y / m_local_costmap_res;
    const int &y = m_local_costmap_height / 2 - pt.x / m_local_costmap_res;
    if(m_c_space_visited[x][y] != 100)
    {
        m_c_space_visited[x][y] = 1;
    }
}

void LocalPlanner::calcOccGrid()
{
    tf::StampedTransform transform;
    tf::TransformListener listener;
    nav_msgs::OccupancyGrid occ_grid;
    int m_grid_array_length = int(m_local_costmap_height * m_local_costmap_width);
    occ_grid.data.clear();
    for(int i = 0; i < m_grid_array_length; i++)
    {
        occ_grid.data.push_back(-1);
    }
    while(!listener.canTransform("/center_laser_link", "/base_link", ros::Time(0)))
    {
        continue;
    }
    try
    {
        listener.lookupTransform("/base_link", "/center_laser_link", ros::Time(0), transform);
    }
    catch(tf::TransformException  ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    occ_grid.header.frame_id = "base_link";
    occ_grid.info.resolution = m_local_costmap_res;
    occ_grid.info.origin.position.x = -m_local_costmap_width * m_local_costmap_res / 2 + transform.getOrigin().getX();
    occ_grid.info.origin.position.y = -m_local_costmap_height * m_local_costmap_res / 2 + transform.getOrigin().getY();
    occ_grid.info.origin.position.z = 0;
    occ_grid.info.origin.orientation.w = transform.getRotation().getW();
    occ_grid.info.origin.orientation.x = transform.getRotation().getX();
    occ_grid.info.origin.orientation.y = transform.getRotation().getY();
    occ_grid.info.origin.orientation.z = transform.getRotation().getZ();
    occ_grid.info.height = m_local_costmap_height;
    occ_grid.info.width = m_local_costmap_width;
    for(int i = 0; i < m_c_space_visited.size(); i++)
    {
        for(int j = 0; j < m_c_space_visited[0].size(); j++)
        {
            auto location = calcGridLocation(Point<int>(i, j));
            if(m_c_space_visited[i][j] == 1)
            {
                occ_grid.data[location] = 100;
            }
        }
    }
    publishOccGrid(occ_grid);
}

void LocalPlanner::pubSpline(const Spline1d &spline)
{
    double it =0;
    double rez = 0.01;
    nav_msgs::Path path;
    path.header.frame_id = "base_link";
    while(it < 1)
    {
        const Eigen::MatrixXd &spline_pt = spline(it);
        const double &x = spline_pt(0);
        const double &y = spline_pt(1);
        it += rez;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        path.poses.push_back(pose);
    }
    m_spline_pub.publish(path);
}

void LocalPlanner::publishOccGrid(const nav_msgs::OccupancyGrid &grid)
{
    m_occ_grid_pub.publish(grid);
}

void LocalPlanner::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    mapCostmapToCSpace(msg);
}

void LocalPlanner::goalPoseCallback(const turtlebot_msgs::GoalPose::ConstPtr &msg)
{
    m_goal_pose = msg;
    planPath();
}

void LocalPlanner::poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    m_pose = msg;
}

}


