/*
 * tag_listener_node.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: Fredrik Macintosh
 *
 *      This program listens to the transforms of tags sent by the ar_track_alvin node and places them in the map frame.
 */


// ******************************************

//



#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf/transform_listener.h"
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <stdio.h>

//****************************************************************************

// Config stuff, muy importante!

std::string frame_id = "/camera_link"; //the frame where you want your marker, the frame in makeMarker needs to be changed manually!

//****************************************************************************


// name of all the transforms received from ar_track_alvar
std::string transNameArray[] = {"/ar_transform_0", "/ar_transform_1", "/ar_transform_2", "/ar_transform_3",
		"/ar_transform_4", "/ar_transform_5"};

// internal names of all the transforms
std::string transMarkerNameArray[] = {"transform_marker_0", "transform_marker_1", "transform_marker_2",
		"transform_marker_3", "transform_marker_4", "transform_marker_5"};

// names of all the markers
std::string markerNameArray[] = {"marker_0", "marker_1", "marker_2",
		"marker_3", "marker_4", "marker_5"};

// array to be filled with corresponding markers
visualization_msgs::MarkerArray markerArray;


// create all the transforms for all the markers and place them in an array
tf::StampedTransform transform_marker_0, transform_marker_1, transform_marker_2, transform_marker_3, transform_marker_4, transform_marker_5;

tf::StampedTransform transArray[] = {transform_marker_0, transform_marker_1, transform_marker_2,
		transform_marker_3, transform_marker_4, transform_marker_5};


// Function that creates the marker.
visualization_msgs::Marker makeMarker(const tf::StampedTransform tagTransform, std::string name,
		int id, int red, int green, int blue, int alpha)
{

	visualization_msgs::Marker marker;

	marker.header.frame_id = "camera_link";  // My frame needs to be changed manually because I don't want a / sign in front.
	marker.header.stamp = ros::Time();
	marker.ns = name;
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = tagTransform.getOrigin().x();
	marker.pose.position.y = tagTransform.getOrigin().y();
	marker.pose.position.z = tagTransform.getOrigin().z() - 0.5; // adjusted to put the box where the real world box is.
	marker.pose.orientation.x = tagTransform.getRotation().x();
	marker.pose.orientation.y = tagTransform.getRotation().y();
	marker.pose.orientation.z = tagTransform.getRotation().z();
	marker.pose.orientation.w = tagTransform.getRotation().w();
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	marker.color.r = red;
	marker.color.g = green;
	marker.color.b = blue;
	marker.color.a = alpha; // alpha = opacity

	return marker;

}// end of makeMarker();


//**********************************************************************************************

int main(int argc, char **argv)
{

	// init the ros node
	ros::init(argc, argv,"tag_listener");

	// create a bunch of node handles that we need
	ros::NodeHandle subNodehandle;
	ros::NodeHandle pubNodehandle;
	ros::NodeHandle tfHandle;

	// create a transform that will be a copy of the transform between map and tag
	tf::TransformListener tagListener;

	// create the publisher of the markerArray
	ros::Publisher markerPublisher = pubNodehandle.advertise<visualization_msgs::MarkerArray>("tag_marker_array", 1000);

	// Set the ros looping rate to 20Hz
	ros::Rate loop_rate(20);

	// Sort of actual main()
	while(ros::ok())
	{
		for (int looper = 0 ; looper < 5 ; looper++)
		{
			// The if test below always returns true, which breaks absolutely everything, needs to be fixed!!!
			if (tagListener.canTransform( frame_id, transNameArray[looper], ros::Time(0)))
			{
				std::cout << "yolo";
				break;
				tagListener.lookupTransform( frame_id, transNameArray[looper], ros::Time(0).now(), transArray[looper]);
				//if (markerArray.markers.size() == 6)
				//{
				//	markerArray.markers.pop_back();
				//	markerArray.markers.push_back(makeMarker(transArray[looper], markerNameArray[looper], 0, 1, 0, 0, 1));
				//}
				//else
				//{
					markerArray.markers.push_back(makeMarker(transArray[looper], markerNameArray[looper], 0, 1, 0, 0, 1));
				//}

			} // end of if
		} // end of for
		markerPublisher.publish(markerArray);

		/**
		 * ros::spinOnce() will pump ONE callback (hence the while).  With this version, all
		 * callbacks will be called from within this thread (the main one).
		 */
		ros::spinOnce();
		loop_rate.sleep();

	}// end of while / sort of actual main()

	return 0;

} // end of main()

