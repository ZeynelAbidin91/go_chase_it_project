#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>

// Define a global client that can rquest services
ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
	ROS_INFO_STREAM("Moving the robot closer to the ball");

	// Request command to move the robot
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	// Call the drive_bot service and pass the requested command
	if (!client.call(srv))
		ROS_ERROR("Failed to call service drive_bot");
}


void process_image_callback(const sensor_msgs::Image img)
{
	int white_pixel = 255;
	int location_pixel;
	
	// Loop through each pixel in the image and check if its equal to the first one
	for (int i=0; i < img.height; i++){
		for (int j=0; j < img.width; j++){
			if (img.data[i,j] == white_pixel)
				location_pixel = j;
			else
				drive_robot(0,0);
		}
	}

	// Find out if the white pixel located in the left, right or middle and request from drive_bot to move tthe robot to the ball
	if (location_pixel <= img.width/3)
		drive_robot(0,0.5);			
	else if (location_pixel > img.width/3 && location_pixel <= img.width*2/3)
		drive_robot(0.5,0);
	else 
		drive_robot(0,-0.5);
		
	
}
			


int main(int argc, char** argv)
{
	//Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from drive_bot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/drive_bot");

	// Subscribe to /rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub = n.subscribe("rgb_camera/image_raw", 10, process_image_callback);

 	// Handle ROS communication events
	ros::spin();

	return 0;
}
