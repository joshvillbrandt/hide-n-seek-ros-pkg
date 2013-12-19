/*
 * File: hide_n_seek/src/planner.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2012
 * Description: This application glues the ROS world and ROMDP class together to seek a person.
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <hide_n_seek/PeopleFinder.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "POMDP.cpp"

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init(argc, argv, "planner");
	ros::NodeHandle nh;
	ros::Rate loop_rate(1); //hz

	// Initialize POMDP
	POMDP pomdp = POMDP();

	// Register publishers
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray> ("hide_n_seek/belief", 1);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_pub("move_base", true);

	// Wait for the action server to come up
	while (!action_pub.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("waiting for the move_base action server to come up");
	}

	// Register subscribers
	ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid> ("/map", 1, &POMDP::updateBeliefFromMap, &pomdp);
	ros::Subscriber people_finder_sub = nh.subscribe<hide_n_seek::PeopleFinder> ("hide_n_seek/people_finder", 1, &POMDP::updateBeliefFromSensor, &pomdp);
	ros::Subscriber robot_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped> ("/robot_pose_ekf/odom_combined", 1, &POMDP::updateRobotPose, &pomdp);

	// Main loop
	while (ros::ok()) {
		// see if it is worth doing a bunch of computation
		if(pomdp.isBeliefUpdated()) {
			pomdp.calculatePolicy();
			marker_pub.publish(pomdp.getBeliefMarkers());
			//marker_pub.publish(pomdp.getPolicyMarkers());
		}

		// send nav goal
		action_pub.sendGoal(pomdp.getNavGoal());

		// update message and enforce rate
		ros::spinOnce();
		loop_rate.sleep();
	}
}
