#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <nav_msgs/Odometry.h>

ros::Subscriber odom_subscriber;

ros::Publisher current_state_publisher;

nav_msgs::Odometry odom;

//this method is used to publish the current state from the odometry

void odomCallback (const nav_msgs::Odometry& odomReceived) {
    odom = odomReceived;
    current_state_publisher.publish(odom);
	ROS_INFO("Current state publisher");
	ROS_INFO("Publishing the current state");
   
}

int main(int argc, char **argv) {
    ros::init(argc,argv, "current_state_publisher"); // name of the node
    ros::NodeHandle nh;

    current_state_publisher = nh.advertise<nav_msgs::Odometry>("/current_state",1);

    odom_subscriber = nh.subscribe("odom", 1, odomCallback);

    ros::spin();
    return 0;
}