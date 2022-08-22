# final_midterm_project


Reference:
Code from Dr.Newman's Repo was used as reference
NOETIC CODE
https://github.com/wsnewman/learning_ros_noetic.git
MELODIC CODE
https://github.com/wsnewman/learning_ros.git

## Running in simulation:
roslaunch mobot_urdf mobot_in_pen.launch

## Running the nodes:

rosrun current_state_publisher current_state_publisher
## This publishes the current state of the robot, this publishes the position and also current state velocities

rosrun lidar_alarm lidar_alarm
## Lidar Alarm which would scan and would also alert if there is any obstacle on the path

rosrun odom_tf odom_tf

rosrun des_pub_state_service des_pub_state_service

##This publishes the desired state of the robot , this also publishes desired state position and desired state velocities



rosrun modal_trajectory_controller lin_steering_with_odom

## This would do the amcl part

rosrun navigation_coordinator navigation_coordinator

##This provides the commands for the robot to move




##Video link: https://drive.google.com/file/d/1Pd4r6FjKtktNDxQsUK1rWak3BtJsIeSH/view?usp=sharing






