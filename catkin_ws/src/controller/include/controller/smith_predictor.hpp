#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <pd_controller.hpp>
#include <pd_ff_controller.hpp>
#include <pid_controller.hpp>
#include <pid_ff_controller.hpp>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <turtlebot_state.hpp>

namespace Turtlebot
{

template <class ContType>

class SmithPredictor        
{   
public:    
    SmithPredictor(const ContType &cont);
    ~SmithPredictor();

    const geometry_msgs::TwistStamped predictControls(const TurtlebotState &current_state,
                                                      const TurtlebotState &desired_state,
                                                      const TurtlebotState &next_desired_state,
                                                      const nav_msgs::Odometry &odom,
                                                      const nav_msgs::Odometry &prev_odom,
                                                      const nav_msgs::Odometry &prev_prev_odom);

private:

    const TurtlebotState predictFeedback(const geometry_msgs::TwistStamped &current_controls,
                                         const nav_msgs::Odometry &odom,
                                         const nav_msgs::Odometry &prev_odom,
                                         const nav_msgs::Odometry &prev_prev_odom);

    ContType *m_cont;
    ContType *m_predict_cont;
    geometry_msgs::TwistStamped m_prev_controls;

    double m_time_delay;
    bool m_first_it = true;
};

}
