
/*
  Software License Agreement (BSD License)

  Copyright (c) 2016, Fredrik Macintosh
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

 * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
 * The name of the original author or Chalmers University of Technology may not be used to endorse or promote products derived
  from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 */




//This program listens to the transforms of tags sent by the modified ar_track_alvin node and places them in a frame of users choice.
// it also publishes the four front-facing corners of a detected "box" for use with the projection mapping.

// If someone who reads this actually know C++ and think I should make header files to handle all my functions, you're probably correct.
// but hey, don't fix what isn't broken.




#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Float64MultiArray.h"

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
#include <ctime>
#include <string>
#include <unistd.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen_conversions/eigen_msg.h>

double pi = 3.1415;

//#include <SFML/Audio.hpp>  Borken sound stuff
//#include <SFML/System.hpp>

//****************************************************************************
// Config stuff, muy importante!
//****************************************************************************

// The frame where you want your marker.
std::string frame_id = "/camera_link";

// Positions of tower blocks to be checked against IRL box position (boxes are cubic, 0.35m^3 -isch  32x35x32...)
//Positions are stored X,Y,Z followed by rotation X, Y, Z, W.


//This array is fugly because I need it to be 4 long for the quaternions
const double posArray[12][4] = {{0,		0,		-1,		0},// Position of first box
		{0,		0.32,	-1,		0}, 						// Position of second box
		{0,		0.32,	-0.68,	0}, 					//
		{0,		0,		-0.68,	0}, 					//
		{0.35,	0,		-1,		0}, 						//
		{-0.35,	0.32,	-1,		0},						// You got it, position of sixth box.

		//Next entries are rotations represented by quaternions, for your own sake, keep x/y/z = 0 and w = 1.
		// Read up on quaternions if you want to be a cool kid and have them something other than zero.

		{0,	0,	0,	1},								//Rotation, in quaternion, of first box
		{0,	0,	0,	1},								//Rotation, in quaternion, of second box
		{0,	0,	0,	1},								//
		{0,	0,	0,	1},								//
		{0,	0,	0,	1},								//
		{0,	0,	0,	1}};							//



// Position error margin for box (radius of error sphere) in meters.
double posHysteresis = 0.05;

//Rotation error margin for box for handling only ONE axis, use: sin(ANGLE_IN_RADIANS/2). For more than 1 axis, you'll have to read about quaternions.
double rotHysteresis = sin((5*pi)/360);

// Sound stuff that doesn't work.
//std::string soundPath = "/home/trykks/catkin_ws/src/tag_listener/sound/";


//****************************************************************************
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

// Keep track of how far into the build we are.
int buildNumber = 1;

// Keep track of whether box is placed correct or not
bool markerPlaced[] = {0,0,0,0,0,0};

// Container of all the front facing corners of all boxes
Eigen::MatrixXd eigenCorners(6,13);

//Keep track of number of positions recieved for each tag
int numberOfDataPoints[6];
//Handle the actual datapoints
double sumOfDataPoints[6][7];
//Store the filters poses of the tags
geometry_msgs::Pose meanPoseArray[6];
//Store all datapoints
double storedDataPoints[6][35];

//Sorts the storedDataPoints[id][] ascending values..
void sortAscending(int id)
{	
	for (int k = 0; k < 6 ; k++)
	{
		//std::cout << "k in bubble sort = " << k << std::endl;
		bool swapped = true;
		double temp;
		while (swapped)
		{
			swapped = false;
			for (int i = 0; i < 4; i++)
			{
				//std::cout << "i in bubble sort = " << i << std::endl;
				if (storedDataPoints[id][i+(k*5)] > storedDataPoints[id][i+(k*5)+1])
				{
					temp = storedDataPoints[id][i+(k*5)];
					storedDataPoints[id][i+(k*5)] = storedDataPoints[id][i+(k*5)+1];
					storedDataPoints[id][i+(k*5)+1] = temp;
					swapped = true;
					
				}
			}
		}
	}
}


// Creates mean values of 5 positions to get a more stable value
bool filterPosition(tf::StampedTransform const tagTransform, int const id)
{
	
	if (numberOfDataPoints[id] < 5)
	{
		storedDataPoints[id][numberOfDataPoints[id]] = tagTransform.getOrigin().x();
		storedDataPoints[id][numberOfDataPoints[id]+5] = tagTransform.getOrigin().y();
		storedDataPoints[id][numberOfDataPoints[id]+10] = tagTransform.getOrigin().z();
		storedDataPoints[id][numberOfDataPoints[id]+15] = tagTransform.getRotation().x();
		storedDataPoints[id][numberOfDataPoints[id]+20] = tagTransform.getRotation().y();
		storedDataPoints[id][numberOfDataPoints[id]+25] = tagTransform.getRotation().z();
		storedDataPoints[id][numberOfDataPoints[id]+30] = tagTransform.getRotation().w();
		numberOfDataPoints[id]++;

		return false;
	} else {
		numberOfDataPoints[id] = 0;
		//std::cout << "Before bubble sort: " << storedDataPoints[id][0] << " " << storedDataPoints[id][1] << " " 
		//	<< storedDataPoints[id][2] << " " << storedDataPoints[id][3] << " " << storedDataPoints[id][4] << " " << storedDataPoints[id][5] << std::endl;
		sortAscending(id);
		//std::cout << "After bubble sort: " << storedDataPoints[id][0] << " " << storedDataPoints[id][1] << " " 
		//	<< storedDataPoints[id][2] << " " << storedDataPoints[id][3] << " " << storedDataPoints[id][4] << " " << storedDataPoints[id][5] << std::endl;
		for (int k = 0; k < 7; k++)
		{
			for (int j = 1; j < 4; j++)
			{
					sumOfDataPoints[id][k] = sumOfDataPoints[id][k] + storedDataPoints[id][j+k*10];
			}
			
			sumOfDataPoints[id][k] = sumOfDataPoints[id][k]/6; 
		}

		meanPoseArray[id].position.x = sumOfDataPoints[id][0];
		meanPoseArray[id].position.y = sumOfDataPoints[id][1];
		meanPoseArray[id].position.z = sumOfDataPoints[id][2];
		meanPoseArray[id].orientation.x = sumOfDataPoints[id][3];
		meanPoseArray[id].orientation.y = sumOfDataPoints[id][4];
		meanPoseArray[id].orientation.z = sumOfDataPoints[id][5];
		meanPoseArray[id].orientation.w = sumOfDataPoints[id][6];
		numberOfDataPoints[id] = 0;
		std::cout << "Filtered y = " << meanPoseArray[id].position.y << std::endl;

		return true;
	}
	
}


// Function that creates the marker.
void makeMarkerArray(tf::StampedTransform const tagTransform, std::string const name,
		double const id, double const red,  double const green, double const blue,  double const alpha , int const i)
{
	markerArray.markers.resize(10);
	markerArray.markers[i].header.frame_id = frame_id;
	markerArray.markers[i].header.stamp = ros::Time(0);
	markerArray.markers[i].ns = name;
	markerArray.markers[i].id = id;
	markerArray.markers[i].type = visualization_msgs::Marker::CUBE;
	markerArray.markers[i].action = visualization_msgs::Marker::ADD;
	// Create a quaternion matrix to be used to correct center of box
	tf::Quaternion quaternion = tagTransform.getRotation();
	// Create vector 0.165 from tag center (boxes are 33cm^3 isch).
	tf::Vector3 corner0 = {-0.23, -0.23, 0}; // sides are 0.34m, distance from center to corner is ~0.23m
	tf::Vector3 corner1 = {-0.23, 0.23, 0};
	tf::Vector3 corner2 = {0.23, -0.23, 0};
	tf::Vector3 corner3 = {0.23, 0.23, 0};
	tf::Vector3 vector (0, 0, 0.165);
	// Rotate the vector by the quaternion to make it follow rotation of tag
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
	markerArray.markers[i].scale.x = 0.34;
	markerArray.markers[i].scale.y = 0.34;
	markerArray.markers[i].scale.z = 0.34;
	markerArray.markers[i].color.r = red;
	markerArray.markers[i].color.g = green;
	markerArray.markers[i].color.b = blue;
	markerArray.markers[i].color.a = alpha; // alpha = opacity

	tf::Vector3 cors[4];
	cors[0]=corner0;
	cors[1]=corner1;
	cors[2]=corner2;
	cors[3]=corner3;

	for(int k=6;k<10;k++){
	
		markerArray.markers[k].scale.x = 0.05;
		markerArray.markers[k].scale.y = 0.05;
		markerArray.markers[k].scale.z = 0.05;
		markerArray.markers[k].color.r = 0;
		markerArray.markers[k].color.g = 0;
		markerArray.markers[k].color.b = 1;
		markerArray.markers[k].color.a = 1; // alpha = opacity

	

	tf::Vector3 correctedVector0 = tf::quatRotate(quaternion, cors[k-6]);

	markerArray.markers[k].header.frame_id = frame_id;
	markerArray.markers[k].header.stamp = ros::Time(0);
	markerArray.markers[k].ns = "name";
	markerArray.markers[k].id = k;
	markerArray.markers[k].type = visualization_msgs::Marker::CUBE;
	markerArray.markers[k].action = visualization_msgs::Marker::ADD;
	markerArray.markers[k].pose.position.x = tagTransform.getOrigin().x() - correctedVector0.x();
	markerArray.markers[k].pose.position.y = tagTransform.getOrigin().y() - correctedVector0.y();
	markerArray.markers[k].pose.position.z = tagTransform.getOrigin().z() - correctedVector0.z();
	markerArray.markers[k].pose.orientation.x = tagTransform.getRotation().x();
	markerArray.markers[k].pose.orientation.y = tagTransform.getRotation().y();
	markerArray.markers[k].pose.orientation.z = tagTransform.getRotation().z();
	markerArray.markers[k].pose.orientation.w = tagTransform.getRotation().w();
	
	}

}// end of makeMarkerArray();


// Tests if box is in correct place
bool boxInCorrectPlace(tf::StampedTransform const transform, int const i)
{
	// TODO ROTATION OF THE BOXES HAS TO BE FULLY HANDLED!! QUATERNIONS AND STUFF.
	

	double posXYZ[7];
	posXYZ[0] = posArray[i][0]; //Positions
	posXYZ[1] = posArray[i][1]; //
	posXYZ[2] = posArray[i][2]; //
	posXYZ[3] = posArray[i][3]; //Rotations (quaternions)
	posXYZ[4] = posArray[i][4]; //
	posXYZ[5] = posArray[i][5]; //
	posXYZ[6] = posArray[i][6]; //

	//std::cout << posXYZ[0] << " " << posXYZ[1] << " " << posXYZ[2] << "\n";

	double errorX;
	double errorY;
	double errorZ;
	//double errorRotX;
	//double errorRotY;
	double errorRotZ;
	//double errorRotW;
	double lengthOfErrorVec;
	//double lengthOfErrorRotVec;

	// Create a quaternion matrix to be used to correct center of box
	tf::Quaternion quaternion = transform.getRotation();
	// Create vector 0.165 from tag center (boxes are 33cm^3 isch).
	tf::Vector3 vector (0, 0, 0.165);
	// Rotate the vector by the quaternion to make it follow rotation of tag
	tf::Vector3 correctedVector = tf::quatRotate(quaternion, vector);

	errorX = (transform.getOrigin().x() - correctedVector.x()) - posXYZ[0]; // Error in X (Corrected to center, same as in makeMarkerArray)
	errorY = (transform.getOrigin().y() - correctedVector.y()) - posXYZ[1]; // Error in Y (Corrected to center, same as in makeMarkerArray)
	errorZ = (transform.getOrigin().z() - correctedVector.z()) - posXYZ[2]; // Error in Z (Corrected to center, same as in makeMarkerArray)

	lengthOfErrorVec = sqrt(pow(errorX,2)+pow(errorY,2)+pow(errorZ,2));

	//errorRotX = transform.getRotation().x() - posXYZ[3];
	//errorRotY = transform.getRotation().y() - posXYZ[4];
	errorRotZ = fabs(transform.getRotation().z() - posXYZ[5]);  // only interested in rotation around the Z-axis, but added full support for the other
	// in case you feel like messing around.
	//errorRotW = transform.getRotation().w() - posXYZ[6];


	//std::cout << lengthOfErrorVec << "\n";
	// Should handle all sides of a box now.. SHOULD
	if ((lengthOfErrorVec < posHysteresis) and ((errorRotZ < rotHysteresis) or
			(errorRotZ < rotHysteresis + 90) or (errorRotZ < rotHysteresis + 180) or (errorRotZ < rotHysteresis + 270)))
		return true;
	else
		return false;

}// End of boxInCorrectPlace()


// Makes corners from the face of the tag.
void fetchCorners(tf::StampedTransform const transform, int i)
{
	// Create a quaternion matrix to be used to correct corners of box
	tf::Quaternion quaternion = transform.getRotation();
	// Create four vectors pointing at corners of box from tag center (assuming box is ~0.34x0.34x0.34m).
	tf::Vector3 corner0 = {-0.23, -0.23, 0}; // sides are 0.34m, distance from center to corner is ~0.23m
	tf::Vector3 corner1 = {-0.23, 0.23, 0};
	tf::Vector3 corner2 = {0.23, -0.23, 0};
	tf::Vector3 corner3 = {0.23, 0.23, 0};
	// Rotate the vectors by the quaternion to make it follow rotation of tag
	tf::Vector3 correctedVector0 = transform.getOrigin() + tf::quatRotate(quaternion, corner0);
	tf::Vector3 correctedVector1 = transform.getOrigin() + tf::quatRotate(quaternion, corner1);
	tf::Vector3 correctedVector2 = transform.getOrigin() + tf::quatRotate(quaternion, corner2);
	tf::Vector3 correctedVector3 = transform.getOrigin() + tf::quatRotate(quaternion, corner3);

	eigenCorners(i,0) = correctedVector0.getX();
	eigenCorners(i,1) = correctedVector0.getY();
	eigenCorners(i,2) = correctedVector0.getZ();

	eigenCorners(i,3) = correctedVector1.getX();
	eigenCorners(i,4) = correctedVector1.getY();
	eigenCorners(i,5) = correctedVector1.getZ();

	eigenCorners(i,6) = correctedVector2.getX();
	eigenCorners(i,7) = correctedVector2.getY();
	eigenCorners(i,8) = correctedVector2.getZ();

	eigenCorners(i,9) = correctedVector3.getX();
	eigenCorners(i,10) = correctedVector3.getY();
	eigenCorners(i,11) = correctedVector3.getZ();

	eigenCorners(i,12) = i;

}

//**********************************************************************************************

int main(int argc, char **argv)
{

	// init the ros node
	ros::init(argc, argv,"tag_listener");

	// create a bunch of node handles that we need
	ros::NodeHandle pubNodehandle;
	ros::NodeHandle pubNodeHandle2;

	// create a transform that will be a copy of the transform between map and tag
	tf::TransformListener tagListener;

	// create the publisher of the markerArray
	ros::Publisher markerPublisher = pubNodehandle.advertise<visualization_msgs::MarkerArray>("tag_marker_array", 100);
	ros::Publisher cornerPublisher = pubNodeHandle2.advertise<std_msgs::Float64MultiArray>("corners", 10);

	// Set the ros looping rate to 20Hz
	ros::Rate loop_rate(20);

	ROS_INFO("Publishing markers to /tag_marker_array");
	ROS_INFO("Publishing corners to /corners");
	ROS_INFO("Listening to transform between %s%s" , frame_id.c_str(), " and ar_transform_N");

	// Sort of actual main()
	while(ros::ok())
	{

		for (int looper = 0 ; looper < 6 ; looper++)
		{

			if (tagListener.canTransform(frame_id, transNameArray[looper], ros::Time(0)))
			{
				//std::cout << "Found a transform! \n";
				try // This try is pretty large, but hey, it works...
				{
					tagListener.waitForTransform(frame_id , transNameArray[looper], ros::Time(0), ros::Duration(0.1));
					tagListener.lookupTransform(frame_id, transNameArray[looper], ros::Time(0), transArray[looper]);
					// Fetch and filter position data here!! Somehow.. change all the transArray[looper] for Pose stuff after filterPosition
					//if (filterPosition(transArray[looper], looper))
					//{
						fetchCorners(transArray[looper], looper);
						for (int k = buildNumber ; k > 0 ; k--)
						{
							if (boxInCorrectPlace(transArray[looper], k-1))
							{
								makeMarkerArray(transArray[looper], markerNameArray[looper], looper, 0, 1 ,0 ,1 ,looper);
								if (!markerPlaced[looper])
								{
									markerPlaced[looper] = true;
									buildNumber++;
								}// end of if
								//std::cout << "Made it to green box markerMaker \n";

							} else // end of if
							{
								//std::cout << "Made it to red box markerMaker \n";
								makeMarkerArray(transArray[looper], markerNameArray[looper], looper, 1, 0 ,0 ,0.5 ,looper);
								markerPlaced[looper] = false;
							}
						}// end of for
					//}	
				}// end of try
				catch (tf::TransformException &ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					continue;
				}

			} // end of if

		} // end of for

		//Convert the Eigen::MatrixXd to a ros message, Float64MultiArray to allow it to be published
		std_msgs::Float64MultiArray stdCorners;
		tf::matrixEigenToMsg(eigenCorners, stdCorners);

		markerPublisher.publish(markerArray);
		cornerPublisher.publish(stdCorners);

		ros::spinOnce();
		loop_rate.sleep();

	}// end of while / sort of actual main()

	return 0;

} // end of main()


