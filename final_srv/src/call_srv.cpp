#include "ros/ros.h"
#include "final_srv/Callbacks_srv.h"

bool callback_srv(final_srv::Callbacks_srv::Request &req, final_srv::Callbacks_srv::Response &res)
{
/* In this function the service handles communications between other nodes by passing values of variables
   or other information, such as which modality the user want to use*/
   	
   	// Information about the status of the goal
	res.s = req.succ; 
	res.re = req.rej;
	
	// Information about the minimum distances on the right, in front and on the left
	res.ri = req.right;
	res.c = req.center;
	res.l = req.left;
	
	// Information about the keyboard command
	res.d = req.dir;
	
	// Information about which controlling modality to use
	res.m1 = req.mod1;
	res.m2 = req.mod2;
		
	return true;
}

int main(int argc, char **argv)
{
	// Initialize the node, setup the NodeHandle for handling the communication with the ROS system
	ros::init(argc, argv, "call_srv");
	ros::NodeHandle ns;
	
	// Define the server for /service custom service
	ros::ServiceServer service = ns.advertiseService("/service", callback_srv);
	
	ros::spin();
	
	return 0;
}
