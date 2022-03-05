#include <ros/ros.h>
#include <traj_builder/traj_builder.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <des_pub_state_service/ServiceMsg.h>
#include <nav_msgs/Odometry.h>
#include <string.h>

using namespace std;

geometry_msgs::Twist g_halt_twist;
geometry_msgs::Twist g_forward_twist;
geometry_msgs::Twist g_spin_twist;
nav_msgs::Odometry g_end_state;
nav_msgs::Odometry g_start_state;
geometry_msgs::PoseStamped g_start_pose;
geometry_msgs::PoseStamped g_end_pose;

bool lidar_alarm = false;
int s = -1; 


const int STOP = 0;
const int FORWARD = 1;
const int SPIN = 2;
const int HALT = 3;
const int BACKUP =4;

ros::Publisher des_state_pub;
ros::Publisher des_twist_pub;
ros::Publisher twist_pub;

ros::Subscriber lidar_sub;
ros::Subscriber current_state_sub;

nav_msgs::Odometry current_state;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_state.pose.pose.orientation.z = -current_state.pose.pose.orientation.z;
    current_state.pose.pose.position.y = -current_state.pose.pose.position.y;
}

bool desStateServiceCallBack(des_pub_state_service::ServiceMsgRequest &request,
                             des_pub_state_service::ServiceMsgResponse &response)
{
    bool success = false;


    int mode = stoi(request.mode);
    s = mode;

    g_start_pose = request.start_pos;
    g_end_pose = request.goal_pos;

    double dt = 0.1;
    ros::Rate looprate(1 / dt);
    TrajBuilder trajBuilder;
    trajBuilder.set_dt(dt);
    trajBuilder.set_alpha_max(0.2);
    trajBuilder.set_accel_max(0.15);
    trajBuilder.set_omega_max(1);
    trajBuilder.set_speed_max(0.8);

    // calculate the desired state stream using traj_builder lib.
    nav_msgs::Odometry des_state;
    des_state.pose.covariance[0] = 0;

    std::vector<nav_msgs::Odometry> vec_of_states;

   
    switch (s)
    {
    case STOP:
        des_state.pose.covariance[0] = STOP;
        des_state.twist.twist.angular.z = 0.0;
        des_state.twist.twist.linear.x = 0.0;
        des_state.header.stamp = ros::Time::now();
        des_state_pub.publish(des_state);
        ros::spinOnce();
        break;

    // FORWARD
    case FORWARD:
        ROS_INFO("GOING FORWARD");
        trajBuilder.build_travel_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.pose.covariance[0] = FORWARD;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
            if (lidar_alarm)
            {
                ROS_INFO("OBSTACLE");
                return response.success = false;;        
            }
        }
        return response.success = true;

    // SPIN
    case SPIN:
        ROS_INFO("GOING SPIN");
        trajBuilder.build_spin_traj(g_start_pose, g_end_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.pose.covariance[0] = SPIN;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        return response.success = true;

    // HALT
    case HALT:
        ROS_INFO("HALT");
        trajBuilder.build_braking_traj(g_start_pose, current_state.twist.twist, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.pose.covariance[0] = HALT;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        return response.success = true;
        
    case BACKUP:
        ROS_INFO("BACKING UP");
        trajBuilder.build_backup_traj(g_start_pose, vec_of_states);
        for (auto state : vec_of_states)
        {
            des_state = state;
            des_state.pose.covariance[0] = BACKUP;
            des_state.header.stamp = ros::Time::now();
            des_state_pub.publish(des_state);
            looprate.sleep();
            ros::spinOnce();
        }
        return response.success = true;
        
    }

    return response.success;
}

void lidarCallback(const std_msgs::Bool &lidar_alarm_recv)
{
    lidar_alarm = lidar_alarm_recv.data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "des_state_publisher_service");
    ros::NodeHandle n;

    ros::ServiceServer des_state_service = n.advertiseService("des_state_publisher_service", desStateServiceCallBack);

    des_state_pub = n.advertise<nav_msgs::Odometry>("/desired_state", 1);

    lidar_sub = n.subscribe("/lidar_alarm", 1, lidarCallback);
    current_state_sub = n.subscribe("/current_state", 1, currStateCallback);\

    ROS_INFO("Ready to publish des_state / des_twist");
    ros::spin();
    return 0;
}
