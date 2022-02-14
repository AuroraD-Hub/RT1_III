#include "ros/ros.h"
#include "move_base_msgs/MoveBaseActionGoal.h" // topic 'move_base/goal'
#include "actionlib_msgs/GoalStatusArray.h" // topic 'move_base/status'
#include "actionlib_msgs/GoalID.h" // topic 'move_base/cancel'
#include "sensor_msgs/LaserScan.h" // topic 'scan'
#include "geometry_msgs/Twist.h" // topic 'cmd_vel'

ros::Publisher pub_goal;
ros::Publisher pub_canc;
ros::Publisher pub_vel;

bool stat[2]; // variable for status of the goal
double min_r, min_c, min_l; // variables for minimum distances scanned from the edges
char dir; // variable for keyboard driving mode

void drive(){
/*In this function the robot moves in the environment while avoiding obstacles*/
	
	double min_th=0.5; // minimum distance threshold from the obstacles	
	geometry_msgs::Twist vel;
	
	if (min_c<min_th){ // obstacle in front of the robot
		ROS_INFO("Cannot go further this way!");
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else{
		if (min_r<min_th){ // obstacle to the right of the robot
			ROS_INFO("Cannot go this way! Turning left...");
			vel.linear.x = 0;
			vel.angular.z = 2;
		}
		else if (min_l<min_th){ // obstacle to the left of the robot
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
	std::cin >> dir;
	
	if (dir=='u'){ // turn left
		vel.linear.x = 0;
		vel.angular.z = 1;
	}
	else if (dir=='i'){ // go foward
		vel.linear.x = 1;
		vel.angular.z = 0;
	}
	else if (dir=='o'){ // turn right
		vel.linear.x = 0;
		vel.angular.z = -1;
	}
	else if (dir=='k'){ // stop
		vel.linear.x = 0;
		vel.angular.z = 0;
	}
	
	pub_vel.publish(vel);
}

void auto_move(){
/* In this modality the robot moves autonomously in the environment to reach a goal 
   passed as input by the user.*/

	move_base_msgs::MoveBaseActionGoal target;
	actionlib_msgs::GoalID canc = {};
	char ui_canc;
	
	ROS_INFO("You chose modality 1! \n");
	
	// Ask the user a new target position
	ROS_INFO("Set x and y of new target position: ");
	std::cin >> target.goal.target_pose.pose.position.x >> target.goal.target_pose.pose.position.y;
	pub_goal.publish(target);
	
	// Drive the robot towards target
	while (stat[0]!=1){
		drive();
	}
	
	/* Control if the target is unreachable and eventually cancel it, otherwise ask the user 
	   if he/she wants to cancel it*/ 
	if (stat[1]==1){ // target is unreachable
		ROS_INFO("Goal cannot be reached!");
		pub_canc.publish(canc);
	}
	else { // asking the user if target has to be cancelled
		ROS_INFO("Do you want to cancel this goal? (Y/N): ");
		std::cin >> ui_canc;
		if (ui_canc=='Y'){ // the user wants to cancel it
			pub_canc.publish(canc);
			ROS_INFO("Goal cancelled");
		}
		else
			ROS_INFO("Ok, going towards the goal");
	}
}

void drive_alone(){
/* In this modality the user is free to entirely control the robot by using the keyboard.*/
	
	ROS_INFO("You chose modality 2! \n");
	
	ROS_INFO("Use keyboard to move the robot:");
	while (dir!='q')
		keyboard_drive();
}

void drive_assistance(){
/* In this modality the user is able to move the robot while is constantly controlled by
   robot itself that acquires information about the environment to avoid collisions.*/
	
	ROS_INFO("You chose modality 3! \n");
	
	ROS_INFO("Use keyboard to move the robot:");
	while (dir!='q'){
		keyboard_drive();
		drive();
	}
}

void callback_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
/* In this function the node constantly get the status of the goal and use it to inform
   the user about it. The 'status_list' array defines 9 possibile states, among which
   state 3 is 'SUCCEDED' and 5 is 'REJECTED': if they are set to '1', it means that the  
   requested target is respectively achieved and unattainable or invalid.*/
   
	stat[0] = msg->status_list[3].status; // SUCCEDED
	stat[1] = msg->status_list[5].status; // REJECTED
}

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
/* In this function the node acquires information from the laser scanner of the robot
   and uses them to move the robot in the circuit. The 'ranges' array, which is of
   size 721 and containes the information needed, is divided in three sections to 
   cover respectively [0 60]° (right section), [60 120]° (central section) and  
   [120 180]° (left section).*/
   	
   	/* Get information about the environment with /base_scan topic: find the minimum
   	   distances from the edges in the three sections. */
	min_r = msg->ranges[0];
	min_c = msg->ranges[240];
	min_l = msg->ranges[480];
	
	// Look for the minimum distances from the nearest obstacles
	for(int i=1; i<=240; i++){
		if(msg->ranges[i]<min_r)
			min_r = msg->ranges[i];
		if(msg->ranges[i+240]<min_c)
			min_c = msg->ranges[i+240];
		if(msg->ranges[i+480]<min_l)
			min_l = msg->ranges[i+480];
	}
}

void ui_decide(){
/* In this function the user can decide which of the available modalities use:
   1) autonomously reach a (x,y) coordinate inserted by the user
   2) let the user drive the robot with the keyboard
   3) let the user drive the robot assisting them to avoid collisions*/
	   
	int mod;
	
	ROS_INFO("Choose among these 3 modalities: \n 1) robot moves autonomously to goal. \n 2) drive the robot to goal. \n 3) drive the robot with assistance to goal.");
	
	std::cin >> mod;
	if (mod==1) // autonomously reach the target
		auto_move();
	else if (mod==2) // user drive the robot
		drive_alone();
	else if (mod==3) // user is assisted to avoid collisions
		drive_assistance();
	else 
		ROS_INFO("This modality doesn't exist.");
}

int main(int argc, char **argv){
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "user_interface");
	ros::NodeHandle nh;

	// Set simulation parameters
	ros::param::set("/goal/target_pose/frame_id", "map");
	ros::param::set("/goal/target_pose/pose/orientation/w", 1);

	// Define the publishers, subscribers and clients to needed topics
	pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
	pub_canc = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
	pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber sub_scan = nh.subscribe("/scan", 1, callback_scan);
	ros::Subscriber sub_status = nh.subscribe("/move_base/status", 1, callback_status);

	// Start the user interface
	while (ros::ok){
		ros::spinOnce();
		
		ui_decide();
	}
	
	return 0;
}
