#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <des_pub_state_service/ServiceMsg.h>
#include <traj_builder/traj_builder.h>

using namespace std;

nav_msgs::Odometry current_state;
geometry_msgs::PoseStamped current_pose;

ros::ServiceClient client;

void currStateCallback(const nav_msgs::Odometry &odom)
{
    current_state = odom;
    current_pose.pose = current_state.pose.pose;
}

void stop(){
    des_pub_state_service::ServiceMsg srv;
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = current_pose;
    srv.request.mode = "0"; 
    if (client.call(srv))
    {
        ROS_INFO("STOP");
    }
  
}

double min_angle(double dang) {
    while (dang > M_PI) dang -= 2.0 * M_PI;
    while (dang < -M_PI) dang += 2.0 * M_PI;
    return dang;
}



float check_angle(float initial_angle,float desired_change)
{   
    double actual_angle = current_state.pose.pose.orientation.z;
    double diff = min_angle(initial_angle+desired_change-actual_angle);
    if (diff>0)
        {
        return diff;
        }
    else
        {
        return -diff;
        }
}


bool rotate(float psi)

{
    //this method just rotates the robot , this is useful for rotating to the respective direction and also amcl
    bool success = true;
    TrajBuilder trajBuilder;
    des_pub_state_service::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;

    bool success_rotate;
  
    ROS_INFO("Rotating at an angle of %f",psi);
  
    // rotate
    goal_pose_rot = trajBuilder.xyPsi2PoseStamped(current_pose.pose.position.x,
                                                  current_pose.pose.position.y,
                                                  psi); 
    srv.request.start_pos = current_pose;
    srv.request.goal_pos = goal_pose_rot;
    srv.request.mode = "2"; 
    if (client.call(srv))
    {
        success_rotate = srv.response.success;
        ROS_INFO("rotate success %d", success_rotate);
    }
    ros::spinOnce();
	return success;
	
}



bool translate(float goal_pose_x, float goal_pose_y)
{	

    bool success = true;
    TrajBuilder trajBuilder;
    des_pub_state_service::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped goal_pose_trans;
    geometry_msgs::PoseStamped goal_pose_rot;
    string mode;
    start_pose.pose = current_state.pose.pose;


    bool success_translate;


    double x_start = start_pose.pose.position.x;
    double y_start = start_pose.pose.position.y;
    double x_end = goal_pose_x;
    double y_end = goal_pose_y;
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double des_psi = atan2(dy, dx);
    rotate(des_psi);



    // forward
    goal_pose_trans = trajBuilder.xyPsi2PoseStamped(goal_pose_x,
                                                    goal_pose_y,
                                                    des_psi); // keep des_psi, change x,y
    srv.request.start_pos = goal_pose_rot;
    srv.request.goal_pos = goal_pose_trans;
    srv.request.mode = "1"; // 
    if (client.call(srv))
    {
        success_translate = srv.response.success;
        ROS_INFO("translate success%d", success_translate);
    }
    ros::spinOnce();

    // if fail to forward
    if (!success_translate)
    {
        ROS_INFO("Cannot move, obstacle. braking");
        srv.request.start_pos = current_pose;
        srv.request.goal_pos = current_pose; 
        srv.request.mode = "3";              
        client.call(srv);
        success = false;
    }
    ros::spinOnce();

    return success;
}

void init_mobot(float goal_pose_x, float goal_pose_y, int retry_max)
{
    int retry_ctr = 0;
    bool success = translate(goal_pose_x, goal_pose_y);
    while (!success && retry_ctr < retry_max) {
        ROS_WARN("RETRY %d", retry_ctr);
        retry_ctr++;
        success = translate(goal_pose_x,goal_pose_y);
    }
}

void backUp()
{
    ROS_INFO("Backing up");
    TrajBuilder trajBuilder;
    des_pub_state_service::ServiceMsg srv;
    geometry_msgs::PoseStamped start_pose;

    start_pose.pose = current_state.pose.pose;

    srv.request.start_pos = current_pose;
    srv.request.goal_pos = current_pose;
    srv.request.mode = "4"; 
    if (client.call(srv))
    {
        bool success_backup = srv.response.success;
        ROS_INFO("backup %d", success_backup);
    }
    ros::spinOnce();
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_coordinator");
    ros::NodeHandle n;

    vector<geometry_msgs::PoseStamped> plan_points;

    client = n.serviceClient<des_pub_state_service::ServiceMsg>("des_state_publisher_service");

    ros::Subscriber current_state_sub = n.subscribe("/current_state", 1, currStateCallback);

    TrajBuilder trajBuilder;

    float x1 = 3.115;
    float y1 = 0.404;

    float x2 = 0.7;
    float y2 = 0.0;

    float x3 = 0.75;
    float y3 = 1.905;

    //float x_o = current_pose.pose.position.x;
    //float y_o = current_pose.pose.position.y;

   ROS_INFO("Rotating for AMCL");
    rotate(0.3);
   ROS_INFO("Rotating for AMCL");
    rotate(-0.3);
  ROS_INFO("Rotating for AMCL");
  rotate(0.4);
  ROS_INFO("Roating for AMCL");
  rotate(-0.3);
    ROS_INFO("Moving to target 1");
    init_mobot(x1, y1, 0);
    
    double prev_angle = current_state.pose.pose.orientation.z;
    backUp();
    rotate(0.5);
    rotate(0.5);
    rotate(0.5);
    double correction = check_angle(prev_angle,1.5);
    rotate(correction);
    
    ROS_INFO("Current state x=[%f],y=[%f]",current_state.pose.pose.position.x,current_state.pose.pose.position.y);
    ROS_INFO("Coming back from target 1");
    init_mobot(2.00,0.15,0);
    init_mobot(1.5,0.05,0);
    init_mobot(x2, y2, 0);
    prev_angle = current_state.pose.pose.orientation.z;
    rotate(0.5);
    rotate(0.57);
    rotate(0.5);
    correction = check_angle(prev_angle,1.57);
    ROS_INFO("Moving to target 2");
    init_mobot(x3, y3, 0);
    
    backUp();

    ROS_INFO("Going home");
    init_mobot(x1, y1, 0);

    //float x_l = current_pose.pose.position.x;
    //float y_l = current_pose.pose.position.y;

    // stop everything
    stop();
    // ROS_INFO("STEP 5");
    // tryMove(-8, current_pose.pose.position.y - 0.05, 1);

    ros::spin();

    return 0;
}
