
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

#define pi 3.1415
#define size2 6


//void sortAscending(int id);

//bool filterPosition(tf::StampedTransform const tagTransform, int const id);


void makeMarkerArray(tf::StampedTransform const tagTransform, std::string const name,
		int const k, double const red,  double const green, double const blue,  double const alpha , int const i, bool const p);

bool boxInCorrectPlace(tf::StampedTransform const transform, int const i);

void fetchCorners(tf::StampedTransform const transform, int eigenIndex, int looper, int p, int buildNumber, bool navCheck);

bool buildPositionVisible (tf::StampedTransform const tagTransform, tf::StampedTransform const robotTransform, int const buildNumber);

//****************************************************************************
// Config stuff, muy importante!
//****************************************************************************

// Positions of tower blocks to be checked against IRL box position (boxes are cubic, 0.35m^3 -isch  32x35x32...)
//Positions are stored X,Y,Z followed by rotation X, Y, Z, W.

//This array is fugly because I need it to be 4 long for the quaternions.
// Make sure you add the correct amount of positions when changing numberOfBoxes
// IMPORTANT: if you ADD positions, you have to change the numberOfPositions accordingly.
const double posArray[12][4] = {{3,		0,		-0.8,		0},	// Position of first box
		{3,		0.345,	-0.8,		0}, 							// Position of second box
		{3-0.345,		0,	-0.8,	0}, 								//
		{3,		0,		-0.8+0.34,	0}, 								//
		{0.345,		0,		-0.8,	0}, 								//
		{0,	0,	-0.8+0.34+0.34,		0},									// You got it, position of sixth box.

		//Next entries are rotations represented by quaternions, for your own sake, keep x/y/z = 0 and w = 1.
		// Read up on quaternions if you want to be a cool kid and have them something other than zero.

		{0,	0,	0,	1},								//Rotation, in quaternion, of first box
		{0,	0,	0,	1},								//Rotation, in quaternion, of second box
		{0,	0,	0,	1},								//
		{0,	0,	0,	1},								//
		{0,	0,	0,	1},								//
		{0,	0,	0,	1}};							//

int numberOfPositions = 6;



// Position error margin for box (radius of error sphere) in meters.
double posHysteresis = 0.1;

//Rotation error margin for box for handling only ONE axis, use: sin(ANGLE_IN_RADIANS/2). For more than 1 axis, you'll have to read about quaternions.
double rotHysteresis = sin((5*pi)/360);

//The number of boxes you want to use
int numberOfBoxes = 1;

// Keep track of how far into the build we are.
int buildNumber = 1;

// The frame where you want your marker.
static std::string frame_id = "/camera_link";


// Keep track of whether box is placed correct or not
std::vector<bool> boxPlaced(1);

// Container of all the front facing corners of all boxes
Eigen::MatrixXd eigenCorners;

//Keep track of number of positions recieved for each tag
//std::vector<int> numberOfDataPoints;
//Handle the actual datapoints
//std::vector<std::vector<double>> sumOfDataPoints;
//Store the filters poses of the tags
//geometry_msgs::Pose meanPoseArray[1];
//Store all datapoints
//std::vector<double> storedDataPoints[1][1];



//This is the fun part, if you want more than 6 boxes, you have to manually add transforms here:

// name of all the transforms received from ar_track_alvar
std::string transNameArray[] = {"/ar_transform_0", "/ar_transform_1", "/ar_transform_2", "/ar_transform_3",
		"/ar_transform_4", "/ar_transform_5"};

// internal names of all the transforms
std::string transMarkerNameArray[] = {"transform_marker_0", "transform_marker_1", "transform_marker_2",
		"transform_marker_3", "transform_marker_4", "transform_marker_5"};

// names of all the markers
std::string markerNameArray[] = {"marker_0", "marker_1", "marker_2",
		"marker_3", "marker_4", "marker_5"};


// create all the transforms for all the markers and place them in an array
tf::StampedTransform transform_marker_0, transform_marker_1, transform_marker_2, transform_marker_3, transform_marker_4, transform_marker_5;

tf::StampedTransform transArray[] = {transform_marker_0, transform_marker_1, transform_marker_2,
		transform_marker_3, transform_marker_4, transform_marker_5};

// You're done here.

//****************************************************************************
//****************************************************************************
  // No touchy!!

int size;

// array to be filled with corresponding markers
visualization_msgs::MarkerArray markerArray;

tf::StampedTransform robotTransform;

//geometry_msgs::Pose buildPose;

visualization_msgs::Marker marker;

geometry_msgs::PoseStamped newCoord;


//**********************************************************************************************

int main(int argc, char **argv)
{
	bool notDone = true;
	std::string navigate;
	// init the ros node
	ros::init(argc, argv,"tag_listener");

	// create a bunch of node handles that we need
	ros::NodeHandle pubNodehandle;

	// create a transform that will be a copy of the transform between map and tag
	tf::TransformListener tagListener;
	tf::TransformListener robotListener;

	markerArray.markers.resize(numberOfPositions + 1);
	size = markerArray.markers.size() - 1;
	

	int eigenIndex = 0;
	std::vector<int> eigenIndexVec(1);
	std::vector<int> boxVisible(0);
	int nrOfVisibleBoxes = 0;

	bool buildPosVis = true;
	bool coordSet = false;



	// create the publisher of the markerArray
	ros::Publisher markerPublisher = pubNodehandle.advertise<visualization_msgs::MarkerArray>("tag_marker_array", 10);
	ros::Publisher cornerPublisher = pubNodehandle.advertise<std_msgs::Float64MultiArray>("corners", 10);
	//ros::Publisher nextBoxPublisher = pubNodehandle.advertise<geometry_msgs::Pose>("nextBoxPose", 10); 
	ros::Publisher newCoordPublisher = pubNodehandle.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
	ros::Publisher goalPublisher = pubNodehandle.advertise<visualization_msgs::Marker>("goal_marker", 1);

	// Set the ros looping rate to 20Hz
	ros::Rate loop_rate(20);
	//Set parameters from console
	std::cout << "****************************************************************************" << std::endl;
	std::cout << "Hello player! \n" << "How many boxes do you want in your tower? Type a number 0-6, then press enter. If you get the idea of choosing anything else, I will break. So please dont." << std::endl;
	std::cin >> numberOfBoxes;
	while (numberOfBoxes == 0)
	{
		std::cout << "You must be a blast at parties... 0-6, go!" << std::endl;
		std::cin >> numberOfBoxes;
		if (numberOfBoxes == 0){
			std::cout << "...really?" << std::endl;
		}
	}
	std::cout << "Cool! \n" << "I will use " << numberOfBoxes << " boxes to build." << std::endl;
	std::cout << "\n \n" << std::endl;
	std::cout << "Do you want to automate the navigation? Please type a simple yes or no, then press enter. If you get the idea of choosing anything else, I will break. So please dont.";
	std::cin >> navigate;
	std::cout << "You answered " << navigate.c_str() << "." << std::endl;
	//std::cout << "What coordinate system do you want to place the boxes in? \n" << "To use the global one, type: /map \n" 
	//<< "To use the Kinect's, type: /camera_link \n" << std::endl;
	//std::cin >> frame_id;

	/*std::time_t timeStart = time(NULL);
	std::time_t timeNow = time(NULL);
	while (abs(timeNow - timeStart) < 6)
	{
		std::cout << "\r" <<"Game will start in " << 6 - (timeNow - timeStart) << " seconds.";
		timeNow = time(NULL);
	}*/
	std::cout << "\n \n" << "Game started! Good luck!" << std::endl;

	//ROS_INFO("Publishing markers to /tag_marker_array");
	//ROS_INFO("Publishing corners to /corners");
	//ROS_INFO("Listening to transform between %s%s" , frame_id.c_str(), " and ar_transform_N");

	ros::Duration d(0.5);
	// Sort of actual main()
	while(ros::ok() && notDone)
	{

		//std::time_t timer = time(NULL);		
		for (int looper = 0 ; looper < 6 ; looper++) // The number 6 is the number of availale tags to loop through, 
		//if not high enough it will look for a specific one. Also, this must not be higher than the length of your markerNameArray() etc
		{
			if (tagListener.canTransform(frame_id, transNameArray[looper], ros::Time(0)) 
				)//&& robotListener.canTransform("/map", "/camera_link", ros::Time(0)))
			{
				//std::cout << "Found a transform! \n";
				try // This try is pretty large, but hey, it works...
				{
					//robotListener.waitForTransform("/map", "/camera_link", ros::Time(0), ros::Duration(0,1));
					//robotListener.lookupTransform("/map", "/camera_link", ros::Time(0), robotTransform);

					tagListener.waitForTransform(frame_id , transNameArray[looper], ros::Time(0), ros::Duration(0.1));
					tagListener.lookupTransform(frame_id, transNameArray[looper], ros::Time(0), transArray[looper]);
					// Fetch and filter position data here!! Somehow.. change all the transArray[looper] for Pose stuff after filterPosition
					//if (filterPosition(transArray[looper], looper))
					//{
					
					if ((ros::Time(0).now().toSec() - transArray[looper].stamp_.toSec()) < 0.25)
					{
						if(!(std::find(boxVisible.begin(), boxVisible.end(), looper) != boxVisible.end()))
						{
							boxVisible.push_back(looper);
							nrOfVisibleBoxes = boxVisible.size();
							//boxVisible.resize(nrOfVisibleBoxes); // Shouldn't be needed because push_back
							boxPlaced.resize(buildNumber+1);
							//eigenIndexVec.resize(nrOfVisibleBoxes);
							eigenCorners.conservativeResize(nrOfVisibleBoxes+1, 14);
						}
	
							for (int k = nrOfVisibleBoxes ; k > 0 ; k--)
							{
								for (int j = buildNumber ; j > 0 ; j--)
								{
									// Need to loop through all build numbers as before
									if (boxInCorrectPlace(transArray[looper], j)) 
									{
										makeMarkerArray(transArray[looper], markerNameArray[looper], j, 0, 1 ,0 ,1 ,looper, false); // move this to after the boxPlaced: Huh??
									
										if (!(std::find(eigenIndexVec.begin(), eigenIndexVec.end(), looper) != eigenIndexVec.end()))
										{
											fetchCorners(transArray[looper], eigenIndex, looper, 1, buildNumber-1, false);
											eigenIndexVec.push_back(looper);
											eigenIndex++;
										}
										buildPosVis = buildPositionVisible(transArray[looper], robotTransform, buildNumber); 
										//std::cout << "looper = "<< looper << std::endl;
										//std::cout << "j - 1 = " << j-1 << std::endl;
										if (!boxPlaced.at(j-1))
										{
											boxPlaced.at(j-1) = true;
											if (buildNumber-1 < numberOfBoxes)
											{
												buildNumber++;
													
											} else {
												notDone = false;
												std::cout << "All done, congratulations!" << std::endl;
											}

										}// end of if(boxPlaced)
										
										else break; // Needs to be here to not break shit. No clue......
									} else if (notDone)// end of if
									{
										//std::cout << "Made it to red box markerMaker \n";

										makeMarkerArray(transArray[looper], markerNameArray[looper], buildNumber, 1, 0 ,0 ,0.5 ,looper, true);
										
										if (!(std::find(eigenIndexVec.begin(), eigenIndexVec.end(), looper) != eigenIndexVec.end()))
										{
											fetchCorners(transArray[looper], eigenIndex, looper, 0, buildNumber-1, false);
											eigenIndexVec.push_back(looper);
											eigenIndex++;
											//std::cout << "eigenIndex = "<< eigenIndex << std::endl;
										}											
										//boxPlaced.at(j-1) = false; //might break shit Edit: Will break shit
									}// end of else if(notDone)
								}// end of for (buildNumber)
							}// end of for (visibleBoxes)
					}// end of if data is the fresh prince of Bel-air	
				}//end of try (transform)
			
	
				catch (tf::TransformException &ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
					continue;
				}

			} // end of if (canTransform)

		} // end of for (looper)
		//std::time_t now = time(NULL);
		//std::cout << "This all took " << now - timer << "s" << std::endl;
		//std::cout << "Boxes visible: " << nrOfVisibleBoxes << std::endl;
		
		//Convert the Eigen::MatrixXd to a ros message, Float64MultiArray to allow it to be published
		std_msgs::Float64MultiArray stdCorners;
		//std::cout << "Didn't crash" << std::endl;

		markerPublisher.publish(markerArray);
		
		goalPublisher.publish(marker);

		if (navigate == "yes"){
			if (!coordSet && !buildPosVis){
				

				eigenCorners.conservativeResize(nrOfVisibleBoxes+2, 14);

				if (robotTransform.getOrigin().getX() < posArray[buildNumber-1][0])
				{
					newCoord.pose.position.x = posArray[buildNumber-1][0]+3;	
				} else {
					newCoord.pose.position.x = posArray[buildNumber-1][0]-3;	
				}
				newCoord.pose.position.y = posArray[buildNumber-1][1];
				newCoord.pose.position.z = posArray[buildNumber-1][2];
				fetchCorners(transArray[0], nrOfVisibleBoxes,1,1,1, true); // only interested in the bool, rest is placeholder
				coordSet = true;
				newCoordPublisher.publish(newCoord);
			}
				
		}

		tf::matrixEigenToMsg(eigenCorners, stdCorners);
		cornerPublisher.publish(stdCorners);
		eigenIndex = 0;
		std::vector<int>().swap(eigenIndexVec);
		std::vector<int>().swap(boxVisible);
		nrOfVisibleBoxes = 0;

		ros::spinOnce();
		loop_rate.sleep();

	}// end of while / sort of actual main()

	return 0;

} // end of main()



//Sorts the storedDataPoints[id][] ascending values..
/*{	
	for (int k = 0; k < numberOfBoxes ; k++)
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

*/

// Creates mean values of 5 positions to get a more stable value
/*bool filterPosition(tf::StampedTransform const tagTransform, int const id)
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
*/

// Function that creates the marker.
void makeMarkerArray(tf::StampedTransform const tagTransform, std::string const name,
		int const k, double const red,  double const green, double const blue,  double const alpha , int const i, bool const p)
{
	//tagTransform = transform of currently detected tag
	//name = name of detected tag
	//k = current buildNumber
	//red = how much red
	//green = how much green
	//blue = how much blue
	//alpha = opacity
	//i = current detected tag
	//p = property false/true (red/green)

	markerArray.markers[i].header.frame_id = frame_id;
	markerArray.markers[i].header.stamp = ros::Time(0);
	markerArray.markers[i].ns = name;
	markerArray.markers[i].id = i;
	markerArray.markers[i].type = visualization_msgs::Marker::CUBE;
	markerArray.markers[i].action = visualization_msgs::Marker::ADD;
	// Create a quaternion matrix to be used to correct center of box
	tf::Quaternion quaternion = tagTransform.getRotation();
	// Create vector 0.165 from tag center (boxes are 33cm^3 isch).
	
	//Debug stuff
	/*tf::Vector3 corner0 = {-0.23, -0.23, 0}; // sides are 0.34m, distance from center to corner is ~0.23m
	tf::Vector3 corner1 = {-0.23, 0.23, 0};
	tf::Vector3 corner2 = {0.23, -0.23, 0};
	tf::Vector3 corner3 = {0.23, 0.23, 0};
	*/

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

if (p == true){



	markerArray.markers[size].header.frame_id = frame_id;	
	markerArray.markers[size].header.stamp = ros::Time(0);
	markerArray.markers[size].ns = "buildGhost";
	markerArray.markers[size].id = 100;
	markerArray.markers[size].type = visualization_msgs::Marker::CUBE;
	markerArray.markers[size].action = visualization_msgs::Marker::ADD;

	markerArray.markers[size].pose.position.x = posArray[k-1][0];
	markerArray.markers[size].pose.position.y = posArray[k-1][1];
	markerArray.markers[size].pose.position.z = posArray[k-1][2];
	
	markerArray.markers[size].pose.orientation.x = posArray[numberOfPositions+(k-1)][0];
	markerArray.markers[size].pose.orientation.y = posArray[numberOfPositions+(k-1)][1];
	markerArray.markers[size].pose.orientation.z = posArray[numberOfPositions+(k-1)][2];
	markerArray.markers[size].pose.orientation.w = posArray[numberOfPositions+(k-1)][3];
	markerArray.markers[size].scale.x = 0.34;
	markerArray.markers[size].scale.y = 0.34;
	markerArray.markers[size].scale.z = 0.34;
	markerArray.markers[size].color.r = 1;
	markerArray.markers[size].color.g = 1;
	markerArray.markers[size].color.b = 0;
	markerArray.markers[size].color.a = 0.5; // alpha = opacity
 	/*
 	buildPose.position.x = posArray[(k-1)][0];
	buildPose.position.y = posArray[(k-1)][1];
	buildPose.position.z = posArray[(k-1)][2];
	buildPose.orientation.x = posArray[numberOfPositions+(k-1)][0];
	buildPose.orientation.y = posArray[numberOfPositions+(k-1)][1];
	buildPose.orientation.z = posArray[numberOfPositions+(k-1)][2];
	buildPose.orientation.w = posArray[numberOfPositions+(k-1)][3];
	*/
}
	
	//Below is for debugging of corners sent

	/*tf::Vector3 cors[4];
	cors[0]=corner0;
	cors[1]=corner1;
	cors[2]=corner2;
	cors[3]=corner3;

	for(int k=numberOfBoxes;k<numberOfBoxes+4;k++){
	
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
	
	} */

}// end of makeMarkerArray();


// Tests if box is in correct place
bool boxInCorrectPlace(tf::StampedTransform const transform, int const i)
{
	//transform = transform of the currently detected tag
	// i = current visible tag

	//TODO: ROTATION OF THE BOXES HAS TO BE HANDLED!! QUATERNIONS AND STUFF.
	double posXYZ[3];
	posXYZ[0] = posArray[i-1][0]; //Positions
	posXYZ[1] = posArray[i-1][1]; //
	posXYZ[2] = posArray[i-1][2]; //
	//posXYZ[3] = posArray[numberOfPositions+(i-1)][0]; //Rotations (quaternions)
	//posXYZ[4] = posArray[numberOfPositions+(i-1)][1]; //
	//posXYZ[5] = posArray[numberOfPositions+(i-1)][2]; //
	//posXYZ[6] = posArray[numberOfPositions+(i-1)][3]; //

	//std::cout << posXYZ[0] << " " << posXYZ[1] << " " << posXYZ[2] << "\n";

	double errorX;
	double errorY;
	double errorZ;
	//double errorRotX;
	//double errorRotY;
	//double errorRotZ;
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
	//std::cout << "buildNumber in boxInCorrectPlace = " << i << std::endl;
	//std::cout << "errorX = " << errorX << " errorY = " << errorY << " errorZ = " << errorZ << std::endl;
	lengthOfErrorVec = sqrt(pow(errorX,2)+pow(errorY,2)+pow(errorZ,2));

	//errorRotX = transform.getRotation().x() - posXYZ[3];
	//errorRotY = transform.getRotation().y() - posXYZ[4];
	//errorRotZ = fabs(transform.getRotation().z() - posXYZ[5]);  // only interested in rotation around the Z-axis, but added full support for the other
	// in case you feel like messing around.
	//errorRotW = transform.getRotation().w() - posXYZ[6];


	//std::cout << lengthOfErrorVec << "\n";
	// Should handle all sides of a box now.. SHOULD
	if ((lengthOfErrorVec < posHysteresis)) {
	// and ((errorRotZ < rotHysteresis) or
			//(errorRotZ < rotHysteresis + 90) or (errorRotZ < rotHysteresis + 180) or (errorRotZ < rotHysteresis + 270)))
	//std::cout << "true" << std::endl;
		return true;
	} else{
		//std::cout << "false" << std::endl;
		return false;
	}

}// End of boxInCorrectPlace()

// Makes corners from the face of the tag and the bottom of it for the buildPosition.
void fetchCorners(tf::StampedTransform const transform, int i, int id, int p, int buildNumber, bool navCheck)
{

	if (!navCheck){
		//transform = transform of the currently detected tag
		//i = current visible tag (isch)
		//p = property of detected box

		// Create a quaternion matrix to be used to correct corners of box
		tf::Quaternion quaternion = transform.getRotation();
		// Create four vectors pointing at corners of box from tag center (assuming box is ~0.34x0.34x0.34m).
		tf::Vector3 corner0 = {-0.165, -0.165, 0}; // sides are 0.34m, distance from center to edge is 0.165m
		tf::Vector3 corner1 = {-0.165, 0.165, 0};
		tf::Vector3 corner2 = {0.165, -0.165, 0};
		tf::Vector3 corner3 = {0.165, 0.165, 0};
		// Rotate the vectors by the quaternion to make it follow rotation of tag
		tf::Vector3 correctedVector0 = transform.getOrigin() + tf::quatRotate(quaternion, corner0);
		tf::Vector3 correctedVector1 = transform.getOrigin() + tf::quatRotate(quaternion, corner1);
		tf::Vector3 correctedVector2 = transform.getOrigin() + tf::quatRotate(quaternion, corner2);
		tf::Vector3 correctedVector3 = transform.getOrigin() + tf::quatRotate(quaternion, corner3);

		// Place all the corners in a matrix	
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

		eigenCorners(i,12) = id; 
		eigenCorners(i,13) = p; //property of corners, used in projection mapping to determine property of projection (red, green etc)

		// Create four vectors pointing at bottom corners of box from tag center (assuming box is ~0.34x0.34x0.34m).
		// x = depth
		// y = width
		// z = height
		corner0 = {-0.165, -0.165, -0.165}; 
		corner1 = {0.165, -0.165, -0.165};
		corner2 = {-0.165, 0.165, -0.165};
		corner3 = {0.165, 0.165, -0.165};

		tf::Vector3 buildCenter = {posArray[buildNumber][0], posArray[buildNumber][1], posArray[buildNumber][2]};

		correctedVector0 = buildCenter + corner0;
		correctedVector1 = buildCenter + corner1;
		correctedVector2 = buildCenter + corner2;
		correctedVector3 = buildCenter + corner3;


		eigenCorners(i+1,0) = correctedVector0.getX();
		eigenCorners(i+1,1) = correctedVector0.getY();
		eigenCorners(i+1,2) = correctedVector0.getZ();

		eigenCorners(i+1,3) = correctedVector1.getX();
		eigenCorners(i+1,4) = correctedVector1.getY();
		eigenCorners(i+1,5) = correctedVector1.getZ();

		eigenCorners(i+1,6) = correctedVector2.getX();
		eigenCorners(i+1,7) = correctedVector2.getY();
		eigenCorners(i+1,8) = correctedVector2.getZ();

		eigenCorners(i+1,9) = correctedVector3.getX();
		eigenCorners(i+1,10) = correctedVector3.getY();
		eigenCorners(i+1,11) = correctedVector3.getZ();

		eigenCorners(i+1,12) = -1; // ID for a buildPosition, used in the projection mapping
		eigenCorners(i+1,13) = 0; // Only ever needs one property ()
	}else{

		// Create four vectors pointing at bottom corners of new navigation position.
		tf::Vector3 corner0 = {-1, -0.165, -1}; 
		tf::Vector3 corner1 = {-1, -0.165, 1};
		tf::Vector3 corner2 = {1, -0.165, 1};
		tf::Vector3 corner3 = {1, -0.165, -1};
		tf::Vector3 navCenter = {newCoord.pose.position.x, newCoord.pose.position.y, newCoord.pose.position.z};

		tf::Vector3 correctedVector0 = navCenter + corner0;
		tf::Vector3 correctedVector1 = navCenter + corner1;
		tf::Vector3 correctedVector2 = navCenter + corner2;
		tf::Vector3 correctedVector3 = navCenter + corner3;


		eigenCorners(i+2,0) = correctedVector0.getX();
		eigenCorners(i+2,1) = correctedVector0.getY();
		eigenCorners(i+2,2) = correctedVector0.getZ();

		eigenCorners(i+2,3) = correctedVector1.getX();
		eigenCorners(i+2,4) = correctedVector1.getY();
		eigenCorners(i+2,5) = correctedVector1.getZ();

		eigenCorners(i+2,6) = correctedVector2.getX();
		eigenCorners(i+2,7) = correctedVector2.getY();
		eigenCorners(i+2,8) = correctedVector2.getZ();

		eigenCorners(i+2,9) = correctedVector3.getX();
		eigenCorners(i+2,10) = correctedVector3.getY();
		eigenCorners(i+2,11) = correctedVector3.getZ();

		eigenCorners(i+2,12) = -2; // ID for a navPosition, used in the projection mapping
		eigenCorners(i+2,13) = 0; // Only ever needs one property ()
	}
}

bool buildPositionVisible (tf::StampedTransform const tagTransform, tf::StampedTransform const robotTransform,
 int const buildNumber)
{
	// Create a quaternion matrix to be used to correct center of box
	tf::Quaternion quaternion = tagTransform.getRotation();
	// Create vector 0.165 from tag center (boxes are 33cm^3 isch).
	tf::Vector3 vector (0, 0, 0.165);
	// Rotate the vector by the quaternion to make it follow rotation of tag
	tf::Vector3 correctedVector = tf::quatRotate(quaternion, vector);

	Eigen::Vector3d tag(correctedVector.getX(), correctedVector.getY(), correctedVector.getZ() );
	Eigen::Vector3d robot( robotTransform.getOrigin().x(), robotTransform.getOrigin().y(), robotTransform.getOrigin().z() );
	Eigen::Vector3d build( posArray[buildNumber-1][0], posArray[buildNumber-1][1], posArray[buildNumber-1][2] );

	Eigen::Vector3d robotToBuild = build-robot;
	Eigen::Vector3d crossProd = (robotToBuild).cross(robot-tag);
	double lengthOfCrossProd = sqrt(pow(crossProd[0], 2) + pow(crossProd[1], 2) + pow(crossProd[2], 2));
	double distBetweenRobAndBuild = sqrt(pow(robotToBuild[0], 2) + pow(robotToBuild[1], 2) + pow(robotToBuild[2], 2));
	double d = lengthOfCrossProd / distBetweenRobAndBuild;
	std::cout << "d = " << d << std::endl;
 	if (d > 0.5)
 		return true;
 	else
 	{
 		
		marker.header.frame_id = "camera_link";
		marker.header.stamp = ros::Time();
		marker.ns = "my_namespace";
		marker.id = 0;
		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 2;
		marker.scale.y = 2;
		marker.scale.z = 0.05;
		marker.color.a = 1.0; // Don't forget to set the alpha!
		marker.color.r = 0.0;
		marker.color.g = 0.0;
		marker.color.b = 1.0;

  		return false;
  	}
}
