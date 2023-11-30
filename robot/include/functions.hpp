#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/WaypointPull.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <vector>
#include <ros/duration.h>
#include <iostream>
#include <string>


mavros_msgs::State current_state;
nav_msgs::Odometry current_pose;
float local_offset;
geometry_msgs::Point local_offset_pose;
float current_heading;
geometry_msgs::PoseStamped waypoint;

ros::Publisher local_pos_pub;
ros::Subscriber current_pos;
ros::Subscriber state_sub;

void state_Cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

int wait4connect()
{
	ROS_INFO("Waiting for FCU connection");
	// wait for FCU connection
	while (ros::ok() && !current_state.connected)
	{
		ros::spinOnce();
		ros::Duration(0.01).sleep();
	}
	if(current_state.connected)
	{
		ROS_INFO("Connected to FCU");	
		return 0;
	}else{
		ROS_INFO("Error connecting to drone");
		return -1;	
	}
	
}

int wait4start()
{
	ROS_INFO("Waiting for user to set mode to GUIDED");
	while(ros::ok() && current_state.mode != "GUIDED")
	{
	    ros::spinOnce();
	    ros::Duration(0.01).sleep();
  	}
  	if(current_state.mode == "GUIDED")
	{
		ROS_INFO("Mode set to GUIDED. Mission starting");
		return 0;
	}else{
		ROS_INFO("Error starting mission!!");
		return -1;	
	}
}

geometry_msgs::Point enu_2_local(nav_msgs::Odometry current_pose_enu)
{
	float x = current_pose_enu.pose.pose.position.x;
	float y = current_pose_enu.pose.pose.position.y;
	float z = current_pose_enu.pose.pose.position.z;
	float deg2rad = (M_PI/180);

	geometry_msgs::Point current_pos_local;

	current_pos_local.x = x*cos((local_offset - 90)*deg2rad) - y*sin((local_offset - 90)*deg2rad);
	current_pos_local.y = x*sin((local_offset - 90)*deg2rad) - y*cos((local_offset - 90)*deg2rad);
	current_pos_local.z = z;

	return current_pos_local;
}

void pose_Cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_pose = *msg;
	enu_2_local(current_pose);
	float q0 = current_pose.pose.pose.orientation.w;
	float q1 = current_pose.pose.pose.orientation.x;
	float q2 = current_pose.pose.pose.orientation.y;
	float q3 = current_pose.pose.pose.orientation.z;
	float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2)+ pow(q3,2))));

	current_heading = psi*(180/M_PI) - local_offset; 
}
int initialize_local_frame()
{
	ROS_INFO("Initialize the local coordinate system");
	local_offset = 0;

	for (int i =1; i<= 30; i++)
	{
		ros::spinOnce();
		ros::Duration(0.1).sleep();

		float q0 = current_pose.pose.pose.orientation.w;
		float q1 = current_pose.pose.pose.orientation.x;
		float q2 = current_pose.pose.pose.orientation.y;
		float q3 = current_pose.pose.pose.orientation.z;
		float psi = atan2((2*(q0*q3 + q1*q2)), (1 - 2*(pow(q2,2)+pow(q3,2)))); //yaw

		local_offset += psi*(180/M_PI);

		local_offset_pose.x = local_offset_pose.x + current_pose.pose.pose.position.x;
		local_offset_pose.y = local_offset_pose.y + current_pose.pose.pose.position.y;
		local_offset_pose.z = local_offset_pose.z + current_pose.pose.pose.position.z;
	}
	local_offset_pose.x = local_offset_pose.x/30 ;
	local_offset_pose.y = local_offset_pose.y/30;
	local_offset_pose.z = local_offset_pose.z/30;
	local_offset /=30;
	ROS_INFO("Coordinate offset set");
	ROS_INFO("the X axis is facing: %f", local_offset);
	return 0;
}

int init_sub_pub(ros::NodeHandle nh)
{
    state_sub = nh.subscribe<mavros_msgs::State>(( "/mavros/state"), 10, state_Cb);

    current_pos = nh.subscribe<nav_msgs::Odometry>(( "/mavros/global_poistion/local"), 10, pose_Cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(("/mavros/setpoint_position/local"),10);

    return 0;
}