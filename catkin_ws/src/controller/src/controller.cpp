#include <controller.hpp>

namespace Turtlebot
{

Controller::Controller(ros::NodeHandle &nh, ros::NodeHandle &pnh, const double &rate) : m_rate(rate)
{
    m_traj_sub = nh.subscribe<turtlebot_msgs::Trajectory>("/local_trajectory", 10, &Controller::trajectoryCallback, this);
    m_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/pf_pose", 10, &Controller::poseCallback, this);
    m_odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom/filtered", 10, &Controller::odomCallback, this);
    m_goal_reached_sub = nh.subscribe<std_msgs::Bool>("/goal_reached", 10, &Controller::goalReachedCallback, this);
    m_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    m_call_type = boost::bind(&Controller::dynamicReconfigureCallback, this, _1, _2);
    m_server = new dynamic_reconfigure::Server<controller::ControllerConfig>(m_config_mutex);
    m_kp_gains_w.resize(5);
    m_ki_gains_w.resize(5);
    m_kd_gains_w.resize(5);
    m_kp_gains_v.resize(5);
    m_ki_gains_v.resize(5);
    m_kd_gains_v.resize(5);
    m_dfl_gains.resize(6);
    m_server->setCallback(m_call_type);
    initializeController(pnh);

    m_error_pos_pub = nh.advertise<turtlebot_msgs::cntrl_pos>("/error_pos_pub", 10);
    m_error_vel_pub = nh.advertise<turtlebot_msgs::cntrl_vel>("/error_vel_pub", 10);
    m_error_acc_pub = nh.advertise<turtlebot_msgs::cntrl_acc>("/error_acc_pub", 10);
    m_error_lin_vel_pub = nh.advertise<turtlebot_msgs::cntrl_lin_vel>("/error_lin_vel_pub", 10);
    m_error_ang_vel_pub = nh.advertise<turtlebot_msgs::cntrl_ang_vel>("/error_ang_vel_pub", 10);

}

Controller::~Controller()
{
    delete m_server;
    delete m_pd_cont;
    delete m_pid_cont;
    delete m_pd_cont;
    delete m_pid_ff_cont;
    delete m_dfl_cont;
}

void Controller::initializeController(ros::NodeHandle &pnh)
{       
    m_server->getConfigDefault(m_config);
    getGains(pnh);
    int type;
    pnh.getParam("controller_type", type);
    if(type == 0)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[0];
        m_config.ki_w = 0;
        m_config.kd_w = m_kd_gains_w[0];
        m_config.kp_v = m_kp_gains_v[0];
        m_config.ki_v = 0;
        m_config.kd_v = m_kd_gains_v[0];
        m_cont_type = controller_type::PD;
        m_pd_cont = new PDController();        
        m_pd_cont->setGains(m_kp_gains_w[0], m_kd_gains_w[0],
                            m_kp_gains_v[0], m_kd_gains_v[0]);
    }
    else if(type == 1)
    {      
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[1];
        m_config.ki_w = m_ki_gains_w[1];
        m_config.kd_w = m_kd_gains_w[1];
        m_config.kp_v = m_kp_gains_v[1];
        m_config.ki_v = m_ki_gains_v[1];
        m_config.kd_v = m_kd_gains_v[1];
        m_cont_type = controller_type::PID;
        m_pid_cont = new PIDController();        
        m_pid_cont->initializeController();
        m_pid_cont->setGains(m_kp_gains_w[1], m_ki_gains_w[1], m_kd_gains_w[1],
                             m_kp_gains_v[1], m_ki_gains_v[1], m_kd_gains_v[1]);
    }
    else if(type == 2)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[2];
        m_config.ki_w = 0;
        m_config.kd_w = m_kd_gains_w[2];
        m_config.kp_v = m_kp_gains_v[2];
        m_config.ki_v = 0;
        m_config.kd_v = m_kd_gains_v[2];
        m_cont_type = controller_type::PD_FF;
        m_pd_ff_cont = new PDFeedForwardController();        
        m_pd_ff_cont->setGains(m_kp_gains_w[2], m_kd_gains_w[2],
                               m_kp_gains_v[2], m_kd_gains_v[2]);
    }
    else if(type == 3)
    {
        m_config.controller_type = type;
        m_config.kp_w = m_kp_gains_w[3];
        m_config.ki_w = m_ki_gains_w[3];
        m_config.kd_w = m_kd_gains_w[3];
        m_config.kp_v = m_kp_gains_v[3];
        m_config.ki_v = m_ki_gains_v[3];
        m_config.kd_v = m_kd_gains_v[3];
        m_cont_type = controller_type::PID_FF;
        m_pid_ff_cont = new PIDFeedForwardController();        
        m_pid_ff_cont->initializeController();
        m_pid_ff_cont->setGains(m_kp_gains_w[3], m_ki_gains_w[3], m_kd_gains_w[3],
                                m_kp_gains_v[3], m_ki_gains_v[3], m_kd_gains_v[3]);
    }
    else if(type == 4)
    {
        m_config.controller_type = type;
        m_config.lam0 = m_dfl_gains[0];
        m_config.lam1 = m_dfl_gains[1];
        m_config.lam2 = m_dfl_gains[2];
        m_config.gam0 = m_dfl_gains[3];
        m_config.gam1 = m_dfl_gains[4];
        m_config.gam2 = m_dfl_gains[5];
        m_cont_type = controller_type::DFL;
        m_dfl_cont = new DYNController();
        m_dfl_cont->setGains(m_dfl_gains[0], m_dfl_gains[1], m_dfl_gains[2],
                             m_dfl_gains[3], m_dfl_gains[4], m_dfl_gains[5]);
    }
    else
    {
        ROS_ERROR("Invalid Controller Type Specified. Options Are: pd, pid, pd_ff, pid_ff, Dyn_feed_lin");
    }
    boost::recursive_mutex::scoped_lock lock(m_config_mutex);
    m_server->updateConfig(m_config);
    lock.unlock();
}

void Controller::getGains(ros::NodeHandle &pnh)
{
    pnh.getParam("kp_w_pd", m_kp_gains_w[0]);
    pnh.getParam("kd_w_pd", m_kd_gains_w[0]);
    pnh.getParam("kp_v_pd", m_kp_gains_v[0]);
    pnh.getParam("kd_v_pd", m_kd_gains_v[0]);
    pnh.getParam("kp_w_pid", m_kp_gains_w[1]);
    pnh.getParam("ki_w_pid", m_ki_gains_w[1]);
    pnh.getParam("kd_w_pid", m_kd_gains_w[1]);
    pnh.getParam("kp_v_pid", m_kp_gains_v[1]);
    pnh.getParam("ki_v_pid", m_ki_gains_v[1]);
    pnh.getParam("kd_v_pid", m_kd_gains_v[1]);
    pnh.getParam("kp_w_pd_ff", m_kp_gains_w[2]);
    pnh.getParam("kd_w_pd_ff", m_kd_gains_w[2]);
    pnh.getParam("kp_v_pd_ff", m_kp_gains_v[2]);
    pnh.getParam("kd_v_pd_ff", m_kd_gains_v[2]);
    pnh.getParam("kp_w_pid_ff", m_kp_gains_w[3]);
    pnh.getParam("ki_w_pid_ff", m_ki_gains_w[3]);
    pnh.getParam("kd_w_pid_ff", m_kd_gains_w[3]);
    pnh.getParam("kp_v_pid_ff", m_kp_gains_v[3]);
    pnh.getParam("ki_v_pid_ff", m_ki_gains_v[3]);
    pnh.getParam("kd_v_pid_ff", m_kd_gains_v[3]);
    pnh.getParam("lam0_dyn_fed_lin", m_dfl_gains[0]);
    pnh.getParam("lam1_dyn_fed_lin", m_dfl_gains[1]);
    pnh.getParam("lam2_dyn_fed_lin", m_dfl_gains[2]);
    pnh.getParam("gam0_dyn_fed_lin", m_dfl_gains[3]);
    pnh.getParam("gam1_dyn_fed_lin", m_dfl_gains[4]);
    pnh.getParam("gam2_dyn_fed_lin", m_dfl_gains[5]);
}

void Controller::control()
{
    if(m_have_pose && m_have_odom && m_have_trajectory && !m_goal_reached && !m_end_of_traj)
    {
        m_current_time = ros::Time::now();
        const TurtlebotState &current_state = getCurrentState();
        const TurtlebotState &desired_state = getDesiredState();
        switch(m_cont_type)
        {
        case PD:
            pubControls(m_pd_cont->getControls(current_state, desired_state));

            m_error_pos_pub.publish(m_pd_cont->m_pos_error);
            m_error_vel_pub.publish(m_pd_cont->m_vel_error);
            m_error_acc_pub.publish(m_pd_cont->m_acc_error);
            m_error_lin_vel_pub.publish(m_pd_cont->m_lin_vel_error);
            m_error_ang_vel_pub.publish(m_pd_cont->m_ang_vel_error);

            break;
        case PID:
            pubControls(m_pid_cont->getControls(current_state, desired_state));

            m_error_pos_pub.publish(m_pid_cont->m_pos_error);
            m_error_vel_pub.publish(m_pid_cont->m_vel_error);
            m_error_acc_pub.publish(m_pid_cont->m_acc_error);
            m_error_lin_vel_pub.publish(m_pid_cont->m_lin_vel_error);
            m_error_ang_vel_pub.publish(m_pid_cont->m_ang_vel_error);

            break;
        case PD_FF:
            pubControls(m_pd_ff_cont->getControls(current_state, desired_state));

            m_error_pos_pub.publish(m_pd_ff_cont->m_pos_error);
            m_error_vel_pub.publish(m_pd_ff_cont->m_vel_error);
            m_error_acc_pub.publish(m_pd_ff_cont->m_acc_error);
            m_error_lin_vel_pub.publish(m_pd_ff_cont->m_lin_vel_error);
            m_error_ang_vel_pub.publish(m_pd_ff_cont->m_ang_vel_error);

            break;
        case PID_FF:
            pubControls(m_pid_ff_cont->getControls(current_state, desired_state));

            m_error_pos_pub.publish(m_pid_ff_cont->m_pos_error);
            m_error_vel_pub.publish(m_pid_ff_cont->m_vel_error);
            m_error_acc_pub.publish(m_pid_ff_cont->m_acc_error);
            m_error_lin_vel_pub.publish(m_pid_ff_cont->m_lin_vel_error);
            m_error_ang_vel_pub.publish(m_pid_ff_cont->m_ang_vel_error);

            break;
        case DFL:
            pubControls(m_dfl_cont->getControls(current_state, desired_state));

            m_error_pos_pub.publish(m_dfl_cont->m_pos_error);
            m_error_vel_pub.publish(m_dfl_cont->m_vel_error);
            m_error_acc_pub.publish(m_dfl_cont->m_acc_error);
            m_error_lin_vel_pub.publish(m_dfl_cont->m_lin_vel_error);
            m_error_ang_vel_pub.publish(m_dfl_cont->m_ang_vel_error);

            break;
        }
    }
    else
    {
        geometry_msgs::TwistStamped null_control;
        null_control.header.stamp = m_current_time;
        null_control.twist.linear.x = 0;
        null_control.twist.linear.y = 0;
        null_control.twist.linear.z = 0;
        null_control.twist.angular.x = 0;
        null_control.twist.angular.y = 0;
        null_control.twist.angular.z = 0;
        pubControls(null_control);
    }
}

const TurtlebotState Controller::getCurrentState()
{
    integrateOdomToCurrentTime();
    integratePoseToCurrentTime();
    const double &x = m_pose_at_control.pose.position.x;
    const double &y = m_pose_at_control.pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_pose_at_control.pose.orientation, q);
    const double &th = tf::getYaw(q);
    const double &v = std::sqrt(std::pow(m_odom_at_control.twist.twist.linear.x, 2)+
                                std::pow(m_odom_at_control.twist.twist.linear.y, 2));
    const double &th_dot = m_odom_at_control.twist.twist.angular.z;
    const double &x_dot = m_odom_at_control.twist.twist.linear.x * cos(th);
    const double &y_dot = m_odom_at_control.twist.twist.linear.x * sin(th);
    const double &dt_prev = ros::Duration(m_odom->header.stamp - m_prev_odom->header.stamp).toSec();
    const double &x_ddot = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) * cos(th) / dt_prev;
    const double &y_ddot = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) * sin(th) / dt_prev;
    const double &dt_prev_prev = ros::Duration(m_prev_odom->header.stamp - m_prev_prev_odom->header.stamp).toSec();
    const double &x_dddot = (x_ddot - (m_prev_odom->twist.twist.linear.x - m_prev_prev_odom->twist.twist.linear.x) * cos(th) / dt_prev_prev) / (dt_prev+dt_prev_prev);
    const double &y_dddot = (y_ddot - (m_prev_odom->twist.twist.linear.x - m_prev_prev_odom->twist.twist.linear.x) * sin(th) / dt_prev_prev) / (dt_prev+dt_prev_prev);
    return TurtlebotState(x, y, th, v, th_dot, x_dot, y_dot, x_ddot, y_ddot, x_dddot, y_dddot);
}

const TurtlebotState Controller::getDesiredState()
{
    const double &start_time = m_traj->header.stamp.toSec();    
    int traj_it = 0;    
    double dt = start_time - m_current_time.toSec();
    while(dt <= 0)
    {
        if(traj_it >= m_traj->durations.size() && !m_goal_reached)
        {
            m_end_of_traj = true;
            return getCurrentState();
        }
        dt += m_traj->durations[traj_it];
        traj_it++;
    }
    const double &time_since_last_traj_pt = m_traj->durations[traj_it] + (m_current_time.toSec() - start_time - m_traj->durations[traj_it] * traj_it);
    return integrateDesiredStateToCurrentTime(traj_it, time_since_last_traj_pt);
}

const TurtlebotState Controller::integrateDesiredStateToCurrentTime(const int &traj_it, const double &dt) const
{
  double acc_ang = 0;
      acc_ang = (m_traj->yaw_rates[traj_it] - m_traj->yaw_rates[traj_it - 1]) / m_traj->durations[traj_it];
      if(std::isnan(acc_ang) || std::isinf(acc_ang))
      {
          acc_ang = 0;
      }
      const double &th_dot = m_traj->yaw_rates[traj_it - 1] + acc_ang * dt;
      const double &th = m_traj->headings[traj_it - 1] + th_dot * dt + acc_ang * std::pow(dt, 2) / 2;
      double jerk_x = 0;
      double jerk_y = 0;
      double acc_x = 0;
      double acc_y = 0;
      if(traj_it < m_traj->durations.size() - 4)
      {
          const double &xp3 = m_traj->x_values[traj_it + 3];
          const double &xp2 = m_traj->x_values[traj_it + 2];
          const double &xp1 = m_traj->x_values[traj_it + 1];
          const double &xp0 = m_traj->x_values[traj_it];
          const double &vxp2 = (xp3 - xp2) / m_traj->durations[traj_it];
          const double &vxp1 = (xp2 - xp1) / m_traj->durations[traj_it];
          const double &vxp0 = (xp1 - xp0) / m_traj->durations[traj_it];
          const double &accxp1 = (vxp2 - vxp1) / m_traj->durations[traj_it];
          acc_x = (vxp1 - vxp0) / m_traj->durations[traj_it];
          jerk_x = (accxp1 - acc_x) / m_traj->durations[traj_it];
          const double &yp3 = m_traj->y_values[traj_it + 3];
          const double &yp2 = m_traj->y_values[traj_it + 2];
          const double &yp1 = m_traj->y_values[traj_it + 1];
          const double &yp0 = m_traj->y_values[traj_it];
          const double &vyp2 = (yp3 - yp2) / m_traj->durations[traj_it];
          const double &vyp1 = (yp2 - yp1) / m_traj->durations[traj_it];
          const double &vyp0 = (yp1 - yp0) / m_traj->durations[traj_it];
          const double &accyp1 = (vyp2 - vyp1) / m_traj->durations[traj_it];
          acc_y = (vyp1 - vyp0) / m_traj->durations[traj_it];
          jerk_y = (accyp1 - acc_y) / m_traj->durations[traj_it];
      }
      const double &acc_lin = std::sqrt(std::pow(acc_x, 2) + std::pow(acc_y, 2));
      const double &v = m_traj->speeds[traj_it - 1] + acc_lin * dt;
      const double &v_x = v * cos(th);
      const double &v_y = v * sin(th);
      const double &x = m_traj->x_values[traj_it - 1] + v_x * dt + acc_x * std::pow(dt, 2) / 2 + jerk_x * std::pow(dt, 3) / 6;
      const double &y = m_traj->y_values[traj_it - 1] + v_y * dt + acc_y * std::pow(dt, 2) / 2 + jerk_y * std::pow(dt, 3) / 6;
      return TurtlebotState(x, y, th, v, th_dot, v_x, v_y, acc_x, acc_y, jerk_x, jerk_y);
}

void Controller::integratePoseToCurrentTime()
{
    ros::Duration dt = ros::Time::now() - m_odom->header.stamp;
    tf::Quaternion q_p;
    tf::quaternionMsgToTF(m_pose->pose.pose.orientation, q_p);
    const double &yaw = tf::getYaw(q_p);
    const double &acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) * cos(yaw) / dt.toSec();
    const double &acc_y = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) * sin(yaw) / dt.toSec();
    const double &acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    const double &v_x = m_odom->twist.twist.linear.x * cos(yaw) + acc_x * dt.toSec();
    const double &v_y = m_odom->twist.twist.linear.x * sin(yaw) + acc_y * dt.toSec();
    const double &v_ang = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    const double &yaw_f = yaw + m_odom->twist.twist.linear.z * dt.toSec() + acc_ang * std::pow(dt.toSec(), 2) / 2;
    double r = std::sqrt(std::pow(v_x, 2) + std::pow(v_y, 2)) / v_ang;
    if(std::isnan(r) || std::isinf(r))
    {
        r = 0;
    }
    const double &x_f = m_pose->pose.pose.position.x - r * sin(yaw) + r * sin(yaw_f);
    const double &y_f = m_pose->pose.pose.position.y + r * cos(yaw) - r * cos(yaw_f);
    m_pose_at_control.pose.position.x = x_f;
    m_pose_at_control.pose.position.y = y_f;
    geometry_msgs::Quaternion q_f;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_f), q_f);
    m_pose_at_control.pose.orientation = q_f;
}

void Controller::integrateOdomToCurrentTime()
{
    ros::Duration dt = m_current_time - m_odom->header.stamp;
    m_odom_at_control.header.stamp = m_current_time;
    const double &acc_x = (m_odom->twist.twist.linear.x - m_prev_odom->twist.twist.linear.x) / dt.toSec();
    const double &acc_y = (m_odom->twist.twist.linear.y - m_prev_odom->twist.twist.linear.y) / dt.toSec();
    const double &acc_ang = (m_odom->twist.twist.angular.z - m_prev_odom->twist.twist.angular.z) / dt.toSec();
    m_odom_at_control.twist.twist.linear.x = m_odom->twist.twist.linear.x + acc_x * dt.toSec();
    m_odom_at_control.twist.twist.linear.y = m_odom->twist.twist.linear.y + acc_y * dt.toSec();
    m_odom_at_control.twist.twist.angular.z = m_odom->twist.twist.angular.z + acc_ang * dt.toSec();
    tf::Quaternion q;
    tf::quaternionMsgToTF(m_odom->pose.pose.orientation, q);
    const double &yaw = tf::getYaw(q);
    const double &yaw_f = yaw + m_odom_at_control.twist.twist.linear.z * dt.toSec();
    double r = m_odom_at_control.twist.twist.linear.x / m_odom_at_control.twist.twist.angular.z;
    if(std::isnan(r) || std::isinf(r))
    {
        r = 0;
    }
    const double &x_f = m_odom->pose.pose.position.x - r * sin(yaw) + r * sin(yaw_f);
    const double &y_f = m_odom->pose.pose.position.y + r * cos(yaw) - r * cos(yaw_f);
    tf::Quaternion q_f = tf::createQuaternionFromYaw(yaw_f);
    q_f.normalize();
    m_odom_at_control.pose.pose.position.x = x_f;
    m_odom_at_control.pose.pose.position.y = y_f;
    geometry_msgs::Quaternion q_;
    tf::quaternionTFToMsg(q_f, q_);
    m_odom_at_control.pose.pose.orientation = q_;
}

void Controller::pubControls(const geometry_msgs::TwistStamped &control) const
{
    m_vel_pub.publish(control.twist);
}

void Controller::dynamicReconfigureCallback(controller::ControllerConfig &config, uint32_t level)
{
    if(config.controller_type != m_config.controller_type)
    {
        const int &type = config.controller_type;
        if(type == 0)
        {
            m_cont_type = Controller::PD;
            m_pd_cont = new PDController;            
            m_pd_cont->setGains(m_kp_gains_w[0], m_kd_gains_w[0],
                                m_kp_gains_v[0], m_kd_gains_v[0]);
            config.kp_w = m_kp_gains_w[0];
            config.ki_w = 0;
            config.kd_w = m_kd_gains_w[0];
            config.kp_v = m_kp_gains_v[0];
            config.ki_v = 0;
            config.kd_v = m_kd_gains_v[0];
        }
        else if(type == 1)
        {
            m_cont_type = Controller::PID;
            m_pid_cont = new PIDController;            
            m_pid_cont->initializeController();
            m_pid_cont->setGains(m_kp_gains_w[1], m_ki_gains_w[1], m_kd_gains_w[1],
                                 m_kp_gains_v[1], m_ki_gains_v[1], m_kd_gains_v[1]);
            config.kp_w = m_kp_gains_w[1];
            config.ki_w = m_ki_gains_w[1];
            config.kd_w = m_kd_gains_w[1];
            config.kp_v = m_kp_gains_v[1];
            config.ki_v = m_ki_gains_v[1];
            config.kd_v = m_kd_gains_v[1];
        }
        else if(type == 2)
        {
            m_cont_type = Controller::PD_FF;
            m_pd_ff_cont = new PDFeedForwardController;            
            m_pd_ff_cont->setGains(m_kp_gains_w[2], m_kd_gains_w[2],
                                   m_kp_gains_v[2], m_kd_gains_v[2]);
            config.kp_w = m_kp_gains_w[2];
            config.ki_w = 0;
            config.kd_w = m_kd_gains_w[2];
            config.kp_v = m_kp_gains_v[2];
            config.ki_v = 0;
            config.kd_v = m_kd_gains_v[2];
        }
        else if(type == 3)
        {
            m_cont_type = Controller::PID_FF;
            m_pid_ff_cont = new PIDFeedForwardController;            
            m_pid_ff_cont->initializeController();
            m_pid_ff_cont->setGains(m_kp_gains_w[3], m_ki_gains_w[3], m_kd_gains_w[3],
                                    m_kp_gains_v[3], m_ki_gains_v[3], m_kd_gains_v[3]);
            config.kp_w = m_kp_gains_w[3];
            config.ki_w = m_ki_gains_w[3];
            config.kd_w = m_kd_gains_w[3];
            config.kp_v = m_kp_gains_v[3];
            config.ki_v = m_ki_gains_v[3];
            config.kd_v = m_kd_gains_v[3];
        }
        else if(type == 4)
        {
            m_cont_type = Controller::DFL;
            m_cont_type = controller_type::DFL;
            m_dfl_cont = new DYNController();
            m_dfl_cont->setGains(m_dfl_gains[0], m_dfl_gains[1], m_dfl_gains[2],
                                 m_dfl_gains[3], m_dfl_gains[4], m_dfl_gains[5]);
            config.lam0 = m_dfl_gains[0];
            config.lam1 = m_dfl_gains[1];
            config.lam2 = m_dfl_gains[2];
            config.gam0 = m_dfl_gains[3];
            config.gam1 = m_dfl_gains[4];
            config.gam2 = m_dfl_gains[5];
        }
    }
}

void Controller::trajectoryCallback(const turtlebot_msgs::Trajectory::ConstPtr &msg)
{
    if(!m_have_trajectory)
    {
        m_have_trajectory = true;
    }
    m_end_of_traj = false;
    m_traj = msg;
}

void Controller::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    if(!m_have_pose)
    {
        m_have_pose = true;
    }
    m_pose = msg;
}

void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(!m_have_odom)
    {
        m_have_odom = true;
        m_odom = msg;
        m_prev_odom = m_odom;
    }
    m_prev_prev_odom = m_prev_odom;
    m_prev_odom = m_odom;
    m_odom = msg;
}

void Controller::goalReachedCallback(const std_msgs::Bool::ConstPtr &msg)
{
    m_goal_reached = msg->data;
    if(m_goal_reached)
    {
        m_have_trajectory = false;
    }
}

}
