#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;
	if (client.call(srv)) 
	{
		ROS_INFO_STREAM("Moving to specified direction");
	}


	else 
	{
		ROS_ERROR("FAILED TO CALL SERVICE");
	}
}

void process_image_callback(const sensor_msgs::Image img)
{
	//get position
	//std::vector<double> current_position = img.position;

	int white_pixel = 255;
	bool ball_in_frame = false;
	int column_index = -1;
	// TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

	for (int i = 0; i < img.height * img.step; i += 3) {
		if (img.data[i] == white_pixel &&
			img.data[i + 1] == white_pixel &&
			img.data[i + 2] == white_pixel)
		{
			ball_in_frame = true;
			column_index = int(i / 3) % img.width; //remainder
			break;
		}
	}
	
	if (!ball_in_frame) //ball not in frame
		drive_robot(0.0, 0.0);
	else
	{
		if (column_index < int(img.width / 3))
		{
			drive_robot(0.0, 0.5); //white pixel in left region, turn left
			ROS_INFO("LEFT");
		}
		else if (column_index >= int(img.width / 3) && column_index <= 2 * int(img.width / 3))//??
		{
			drive_robot(0.5, 0); //forward
			ROS_INFO("FORWARD");
		}
		else
		{
			drive_robot(0.0, -0.5);
			ROS_INFO("RIGHT");
		}
    }
}

int main(int argc, char** argv)
{
	// Initialize the process_image node and create a handle to it
	ros::init(argc, argv, "process_image");
	ros::NodeHandle n;

	// Define a client service capable of requesting services from command_robot
	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();

	return 0;
}
