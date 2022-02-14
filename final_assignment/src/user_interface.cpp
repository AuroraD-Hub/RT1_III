#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"
#include "move_base_msgs/MoveBaseActionGoal.h" // topic 'move_base/goal'
#include "actionlib_msgs/GoalID.h" // topic 'move_base/cancel'

ros::Publisher pub_goal;
ros::Publisher pub_canc;
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

void auto_move(){
/* In this modality the robot moves autonomously in the environment to reach a goal 
   passed as input by the user.*/

	move_base_msgs::MoveBaseActionGoal target;
	actionlib_msgs::GoalID canc = {};
	char ui_canc;
	
	ROS_INFO("You chose modality 1! \n");
	
	//client.call(var);
	get_var();
	
	// Ask the user a new target position
	ROS_INFO("Set x and y of new target position: ");
	std::cin >> target.goal.target_pose.pose.position.x >> target.goal.target_pose.pose.position.y;
	pub_goal.publish(target);
	
	// Drive the robot towards target
	while (var.response.s!=1){
		var.request.mod1 = 0;
		client.call(var);
	}
	
	/* Control if the target is unreachable and eventually cancel it, otherwise ask the user 
	   if he/she wants to cancel it*/ 
	if (var.response.re==1){ // target is unreachable
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
	
	//client.call(var);
	get_var();
	
	ROS_INFO("Use keyboard to move the robot:");
	while (var.response.d!='q'){
		var.request.mod2 = 1;
		client.call(var);
	}
}

void drive_assistance(){
/* In this modality the user is able to move the robot while is constantly controlled by
   robot itself that acquires information about the environment to avoid collisions.*/
	
	ROS_INFO("You chose modality 3! \n");
	
	//client.call(var);
	get_var();
	
	ROS_INFO("Use keyboard to move the robot:");
	while (var.response.d!='q'){
		var.request.mod1 = 0;
		var.request.mod2 = 1;
		client.call(var);
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
	client = nh.serviceClient<final_srv::Callbacks_srv>("/service");
	
	// Start the user interface
	while (ros::ok){
		ui_decide();
	}
	
	return 0;
}
