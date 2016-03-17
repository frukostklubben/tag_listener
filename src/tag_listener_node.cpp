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
#include "tf/tf.h"
#include <complex>
#include <math.h>
#include <cmath>



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
void makeMarkerArray(tf::StampedTransform const tagTransform, std::string const name,
	int const id, int const red,  int const green, int const blue,  double const alpha , int const i)

{

	markerArray.markers.resize(6);
	markerArray.markers[i].header.frame_id = "camera_link";  // My frame needs to be changed manually because I don't want a / sign in front.
	markerArray.markers[i].header.stamp = ros::Time(0);
	markerArray.markers[i].ns = name;
	markerArray.markers[i].id = id;
	markerArray.markers[i].type = visualization_msgs::Marker::CUBE;
	markerArray.markers[i].action = visualization_msgs::Marker::ADD;
	// Create a quaternion matrix to be used to correct center of box
	tf::Quaternion quaternion = tagTransform.getRotation();
	// Create vector 0.25 from tag center.
	tf::Vector3 vector (0, 0, 0.25);
	// Rotate the vector by the quaternion to correct it
	tf::Vector3 correctedVector = tf::quatRotate(quaternion, vector);

	// Add (Subtract) the new corrected vector to the position of the marker to place it at the correct place in space.
	markerArray.markers[i].pose.position.x = tagTransform.getOrigin().x() - correctedVector.x();
	markerArray.markers[i].pose.position.y = tagTransform.getOrigin().y() - correctedVector.y();
	markerArray.markers[i].pose.position.z = tagTransform.getOrigin().z() - correctedVector.z();
	// ----------------------------------------------------------------------------------------------------------
	markerArray.markers[i].pose.orientation.x = tagTransform.getRotation().x();
	markerArray.markers[i].pose.orientation.y = tagTransform.getRotation().y();
	markerArray.markers[i].pose.orientation.z = tagTransform.getRotation().z();
	markerArray.markers[i].pose.orientation.w = tagTransform.getRotation().w();
	markerArray.markers[i].scale.x = 0.5;
	markerArray.markers[i].scale.y = 0.5;
	markerArray.markers[i].scale.z = 0.5;
	markerArray.markers[i].color.r = red;
	markerArray.markers[i].color.g = green;
	markerArray.markers[i].color.b = blue;
	markerArray.markers[i].color.a = alpha; // alpha = opacity

}// end of makeMarkerArray();



//**********************************************************************************************

int main(int argc, char **argv)
{

	// init the ros node
	ros::init(argc, argv,"tag_listener");

	// create a bunch of node handles that we need
	ros::NodeHandle pubNodehandle;
	ros::NodeHandle tfHandle;

	// create a transform that will be a copy of the transform between map and tag
	tf::TransformListener tagListener;

	// create the publisher of the markerArray
	ros::Publisher markerPublisher = pubNodehandle.advertise<visualization_msgs::MarkerArray>("tag_marker_array", 10);

	// Set the ros looping rate to 20Hz
	ros::Rate loop_rate(20);


	// Sort of actual main()
	while(ros::ok())
	{
		std::cout << iterationArray[0];
		std::cout << " ";
		std::cout << iterationArray[1];
		std::cout << " ";
		std::cout << iterationArray[2];
		std::cout << " ";
		std::cout << iterationArray[3];
		std::cout << " ";
		std::cout << iterationArray[4];
		std::cout << " ";
		std::cout << iterationArray[5];
		std::cout << "\n";
		for (int looper = 0 ; looper < 6 ; looper++)
		{
			
			if (tagListener.canTransform( frame_id, transNameArray[looper], ros::Time(0)))
			{
				iterationArray[looper]++;
			} else if (iterationArray[looper] > 0){
				iterationArray[looper]--;
			}
			if (iterationArray[looper] > 30)
			{
				try
				{
					tagListener.waitForTransform(frame_id , transNameArray[looper], ros::Time(0), ros::Duration(0.1));

					tagListener.lookupTransform(frame_id, transNameArray[looper], ros::Time(0), transArray[looper]);

				}
				catch (tf::TransformException &ex)
				{
      				ROS_ERROR("%s",ex.what());
      				ros::Duration(1.0).sleep();
      				continue;
    			}
    			makeMarkerArray(transArray[looper], markerNameArray[looper], looper, 1, 0 ,0 ,0.5 ,looper);
    			for (int k = 0 ; k < 6 ; k++)
    			{
    			}
			} // end of if

		} // end of for
		markerPublisher.publish(markerArray);

		ros::spinOnce();
		loop_rate.sleep();

	}// end of while / sort of actual main()

	return 0;

} // end of main()

