#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"
#include <std_msgs/Float64.h>

ros::Publisher motor_command_publisher;

bool handle_drive_bot_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{

	ROS_INFO(" DriveToTargetRequest received - linear x:%1.2f, angular z:%1.2f",(float)req.linear_x, (float)req.angular_z);

	// Define the message to publish on topic to move the robot	
	geometry_msgs::Twist motor_command;

	motor_command.linear.x = req.linear_x;
	motor_command.angular.z = req.angular_z;

	// Publish drive commands to the robot
	motor_command_publisher.publish(motor_command);
	
	// Wait 3 seconds for robot to move
	ros::Duration(3).sleep();

	// 
	res.msg_feedback = "Linear and angular velocity set - linear x:%1.2f, angular z:%1.2f",(float)req.linear_x, (float)req.angular_z;
	ROS_INFO_STREAM(res.msg_feedback);
		
	return true;
}

int main(int argc, char** argv)
{
	// Initialize the drive_bot node and create a handle to it
	ros::init(argc, argv, "drive_bot");
	ros::NodeHandle n;

	// Define two publishers to publish std_msgs::Float64 messahges on robot's velocity topic
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel",10);

	// Define a drive_bot service with a handle_drive_bot_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_bot_request);
	ROS_INFO("Ready to send drive commands");

	// Handle ROS communication events
	ros::spin();
	
	return 0;
}
