#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"
#include "geometry_msgs/Twist.h" // topic 'cmd_vel'

ros::Publisher pub_vel;
ros::ServiceClient client;

final_srv::Callbacks_srv var;

void get_var(){
	
	client.call(var);
	
	// Information about the status of the goal
	var.response.s = var.request.succ; 
	var.response.re = var.request.rej;
	
	// Information about the minimum distances on the right, in front and on the left
	var.response.ri = var.request.right;
	var.response.c = var.request.center;
	var.response.l = var.request.left;
	
	// Information about the keyboard command
	var.response.d = var.request.dir;
	
	// Information about which controlling modality to use
	var.response.m1 = var.request.mod1;
	var.response.m2 = var.request.mod2;
}

void drive(){
/*In this function the robot moves in the environment while avoiding obstacles*/
	
	double min_th=0.5; // minimum distance threshold from the obstacles	
	geometry_msgs::Twist vel;
	
	//client.call(var);
	get_var();
	
	if (var.response.c<min_th){ // obstacle in front of the robot
		ROS_INFO("Cannot go further this way!");
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else{
		if (var.response.ri<min_th){ // obstacle to the right of the robot
			ROS_INFO("Cannot go this way! Turning left...");
			vel.linear.x = 0;
			vel.angular.z = 2;
		}
		else if (var.response.l<min_th){ // obstacle to the left of the robot
			ROS_INFO("Cannot go this way! Turning right...");
			vel.linear.x = 0;
			vel.angular.z = -2;
		}
	}
	pub_vel.publish(vel);
}

void keyboard_drive(){
/*In this function user drives the robot in the environment by keyboard command*/
	
	geometry_msgs::Twist vel;
	
	ROS_INFO("Press 'u' to turn left. \n Press 'i' to go foward \n Press 'o' to turn right \n Press 'k' to stop \n Press 'q' to quit this modality");
	std::cin >> var.request.dir;
	
	client.call(var);
	get_var();
	
	if (var.response.d=='u'){ // turn left
		vel.linear.x = 0;
		vel.angular.z = 1;
	}
	else if (var.response.d=='i'){ // go foward
		vel.linear.x = 1;
		vel.angular.z = 0;
	}
	else if (var.response.d=='o'){ // turn right
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else if (var.response.d=='k'){ // stop
		vel.linear.x = 0;
		vel.angular.z = 0;
	}
	
	pub_vel.publish(vel);
}

int main(int argc, char **argv){
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "driver");
	ros::NodeHandle nh;

	// Set simulation parameters
	ros::param::set("/goal/target_pose/frame_id", "map");
	ros::param::set("/goal/target_pose/pose/orientation/w", 1);

	// Define the publishers, subscribers and clients to needed topics
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	client = nh.serviceClient<final_srv::Callbacks_srv>("/service");
	
	// Start the user interface
	//client.call(var);
	get_var();
	if (var.response.m1==0)
		drive();
	else if (var.response.m2==1)
		keyboard_drive();
	
	return 0;
}
