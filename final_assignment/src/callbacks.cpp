#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"
#include "actionlib_msgs/GoalStatusArray.h" // topic 'move_base/status'
#include "sensor_msgs/LaserScan.h" // topic 'scan'

ros::ServiceClient client;
final_srv::Callbacks_srv var;

void callback_scan(const sensor_msgs::LaserScan::ConstPtr& msg){
/* In this function the node acquires information from the laser scanner of the robot
   and uses them to move the robot in the circuit. The 'ranges' array, which is of
   size 721 and containes the information needed, is divided in three sections to 
   cover respectively [0 60]° (right section), [60 120]° (central section) and  
   [120 180]° (left section).*/
   	
   	/* Get information about the environment with /base_scan topic: find the minimum
   	   distances from the edges in the three sections. */
	var.request.right = msg->ranges[0];
	var.request.center = msg->ranges[240];
	var.request.left = msg->ranges[480];
	
	// Look for the minimum distances from the nearest obstacles
	for(int i=1; i<=240; i++){
		if(msg->ranges[i]<var.request.right)
			var.request.right = msg->ranges[i];
		if(msg->ranges[i+240]<var.request.center)
			var.request.center = msg->ranges[i+240];
		if(msg->ranges[i+480]<var.request.left)
			var.request.left = msg->ranges[i+480];
	}
	
	client.call(var);
}

void callback_status(const actionlib_msgs::GoalStatusArray::ConstPtr& msg){
/* In this function the node constantly get the status of the goal and use it to inform
   the user about it. The 'status_list' array defines 9 possibile states, among which
   state 3 is 'SUCCEDED' and 5 is 'REJECTED': if they are set to '1', it means that the  
   requested target is respectively achieved and unattainable or invalid.*/

	var.request.succ = msg->status_list[3].status; // SUCCEDED
	var.request.rej = msg->status_list[5].status; // REJECTED
	
	client.call(var);
}

int main(int argc, char **argv){
	
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "callbacks");
	ros::NodeHandle nh;
	
	// Set simulation parameters
	ros::param::set("/goal/target_pose/frame_id", "map");
	ros::param::set("/goal/target_pose/pose/orientation/w", 1);
	
	// Define the publishers, subscribers and clients to needed topics
	ros::Subscriber sub_scan = nh.subscribe("/scan", 1, callback_scan);
	ros::Subscriber sub_status = nh.subscribe("/move_base/status", 1, callback_status);
	client = nh.serviceClient<final_srv::Callbacks_srv>("/service");
	
	ros::spin();

	return 0;
}
