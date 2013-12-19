/*
 * File: hide_n_seek/src/fake_people_finder.cpp
 * Author: Josh Villbrandt <josh@javconcepts.com>
 * Date: April 2012
 * Description: This application identified a person in a simulated world. The person is actually an rviz interactive marker.
 */

#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <hide_n_seek/PeopleFinder.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_listener.h>
#include <time.h>

#define PI 3.14159265
const float sensor_range = 3.5; //m
const float sensor_fov = 57*PI/180; //~1 radian
const float percent_true_detected = 90; //
const float percent_true_undetected = 5; // false positives
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher pub_people_finder;
ros::Publisher pub_sensor_shadow;
nav_msgs::GridCells sensor_shadow;
geometry_msgs::PoseStamped fakePersonPoseMap;

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
		<< feedback->pose.position.x << ", " << feedback->pose.position.y
		<< ", " << feedback->pose.position.z );

	// save for future reference
	fakePersonPoseMap.pose = feedback->pose;
}

void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	geometry_msgs::Pose pose = feedback->pose;

	pose.position.x = round(pose.position.x-0.5)+0.5;
	pose.position.y = round(pose.position.y-0.5)+0.5;

	//ROS_INFO_STREAM( feedback->marker_name << ":"
	//  << " aligning position = "
	//  << feedback->pose.position.x
	//  << ", " << feedback->pose.position.y
	//  << ", " << feedback->pose.position.z
	//  << " to "
	//  << pose.position.x
	//  << ", " << pose.position.y
	//  << ", " << pose.position.z );

	server->setPose( feedback->marker_name, pose );
	server->applyChanges();

	// save for future reference
	fakePersonPoseMap.pose = pose;
}

visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg) {
	visualization_msgs::Marker marker;

	marker.type = visualization_msgs::Marker::CUBE;
	marker.scale.x = 0.45;
	marker.scale.y = 0.45;
	marker.scale.z = 0.45;
	// orange is my favorite color
	marker.color.r = 1.0;
	marker.color.g = 0.45;
	marker.color.b = 0.0;
	marker.color.a = 1.0;

	return marker;
}

void initializeMarker() {
	// create an interactive marker server on the topic namespace simple_marker
	server.reset( new interactive_markers::InteractiveMarkerServer("fake_people_finder","",false) );

	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "/map";
	int_marker.name = "fake_person";
	int_marker.description = "\t\t\t\tFake Person\n(slide me somewhere in the map!)";
	int_marker.pose.position.x = 0;
	int_marker.pose.position.y = -7.5;

	visualization_msgs::InteractiveMarkerControl control;
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
	int_marker.controls.push_back(control);

	// make a box which also moves in the plane
	control.markers.push_back( makeBox(int_marker) );
	control.always_visible = true;
	int_marker.controls.push_back(control);

	// we want to use our special callback function
	server->insert(int_marker);
	server->setCallback(int_marker.name, &processFeedback);

	// set different callback for POSE_UPDATE feedback
	//server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

	// 'commit' changes and send to all clients
	server->applyChanges();
}

void initializeSensorShadow() {
	sensor_shadow.header.stamp = ros::Time::now();
	sensor_shadow.header.frame_id = "base_link";
	sensor_shadow.cell_width = 0.1;
	sensor_shadow.cell_height = 0.1;

	// reuse this point
	geometry_msgs::Point point;
	point.z = 0;

	// add sides
	float half_theta = tan(sensor_fov / 2);
	for(int i = 0; i < 10*sensor_range; i++) {
		float range = 0.1 * i;
		point.x = range * cos(half_theta);
		point.y = range * sin(half_theta);
		sensor_shadow.cells.push_back(point);
		point.y = -1 * point.y;
		sensor_shadow.cells.push_back(point);
	}

	// add tip
	for(int i = 0; i < 40*(sensor_fov+0.1)/2; i++) {
		float theta = 0.025 * i;
		point.x = sensor_range * cos(theta);
		point.y = sensor_range * sin(theta);
		sensor_shadow.cells.push_back(point);
		point.y = -1 * point.y;
		sensor_shadow.cells.push_back(point);
	}
}

void findPeople(const tf::TransformListener& tf_listener) {
	// initialize message
	hide_n_seek::PeopleFinder people_msg;
	people_msg.header.stamp = ros::Time();
	people_msg.header.frame_id = "base_link";
	people_msg.sensor_range = sensor_range;
	people_msg.sensor_fov = sensor_fov;
	bool foundPerson = false;

	try {
		fakePersonPoseMap.header.stamp = ros::Time(); // trust me, it is still there
		fakePersonPoseMap.header.frame_id = "/map";

		// transform
		geometry_msgs::PoseStamped fakePersonPoseBase;
		tf_listener.transformPose ("base_link", fakePersonPoseMap, fakePersonPoseBase);
		//ROS_INFO_STREAM( " fake person = "
		//  << fakePersonPoseBase.pose.position.x
		//  << ", " << fakePersonPoseBase.pose.position.y
		//  << ", " << fakePersonPoseBase.pose.position.z);

		// search for people
		float range = sqrt(pow(fakePersonPoseBase.pose.position.x,2) + pow(fakePersonPoseBase.pose.position.y,2));
		if(fakePersonPoseBase.pose.position.x > 0 && range <= sensor_range) {
			if(fakePersonPoseBase.pose.position.y == 0) foundPerson = true;
			else {
				float bearing = PI/2 - atan(fakePersonPoseBase.pose.position.x/fabs(fakePersonPoseBase.pose.position.y));
				if(bearing < sensor_fov/2) foundPerson = true;
				//ROS_INFO_STREAM(bearing);
			}
		}

		// make sensor reading noisy
		int randNum = rand() % 100;
		if(foundPerson && randNum > percent_true_detected) foundPerson = false;
		else if(!foundPerson && randNum <= percent_true_undetected) {
			foundPerson = true;
			fakePersonPoseBase.pose.position.x = sensor_range / 2 * cos(tan(sensor_fov / 4));
			fakePersonPoseBase.pose.position.y = sensor_range / 2 * sin(tan(sensor_fov / 4));
		}
		if(foundPerson) people_msg.people.push_back(fakePersonPoseBase.pose); // this is more realistic (Kinect node would give relative position)
	}
	catch (tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

	// publish current sensor shadow
	pub_sensor_shadow.publish(sensor_shadow);

	// publish any people we found
	pub_people_finder.publish(people_msg);
	ROS_INFO_STREAM("found " << people_msg.people.size() << " people");
}

int main(int argc, char** argv) {
	// Initialize ROS
	ros::init (argc, argv, "fake_people_finder");
	ros::NodeHandle nh;
	ros::Rate loop_rate(2);
	tf::TransformListener tf_listener;

	// initialize random seed
	srand( time(NULL) );

	// create a rviz marker to act as the fake person
	initializeMarker();

	// Create ROS publishers
	pub_people_finder = nh.advertise<hide_n_seek::PeopleFinder>("hide_n_seek/people_finder", 1);
	pub_sensor_shadow = nh.advertise<nav_msgs::GridCells>("hide_n_seek/sensor_shadow", 1);

	// create sensor shadow (this is constant)
	initializeSensorShadow();

	// Spin
	while(ros::ok()) {
		findPeople(boost::ref(tf_listener));

		ros::spinOnce();
		loop_rate.sleep();
	}
}
