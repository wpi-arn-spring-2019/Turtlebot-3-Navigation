# Turtlebot-3-Navigation

Full stack navigation implementation for turtlebot 3


DEPENDENCIES:

ROS Kinetic, PCL, Eigen, turtlebot3*

compile with catkin_make

simulation: roslaunch bringup full_sim.launch

real robot: roslaunch bringup real_robot.launch

not: real robot is configured to load custom real map, change this in real_robot.launch to load a different map

give a 2d pose estimate in rviz, then a 2d nav goal
