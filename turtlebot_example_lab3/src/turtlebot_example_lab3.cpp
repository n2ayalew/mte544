//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <eigen3/Eigen/Dense>

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

/** Example usage of std::normal_distribution
#define MEAN 10.0
#define STDDEV 2.0

std::default_random_engine generator; //setup a random number generator
std::normal_distribution<double> distribution(MEAN,STDDEV); //setup the normal distribution
double number = distribution(generator); //get random number
**/

#define TAGID 0
#define NUM_SAMPLES 1000
#define OCCUPIED_THRESHOLD 1
#define NEAREST_EDGES 8

using namespace Eigen;

typedef struct pose {
	double x;
	double y;
	double theta;
} pose_t;

typedef struct mapData {
	float res;
	uint32_t w;
	uint32_t h;
	struct pose origin;
	std::string frame_id;
	//geometry_msgs::Pose; left here just in case we need it
} mapData_t;

ros::Publisher marker_pub;
bool mapRecieved = false;
char **occupancyGrid; // map represented as occupancy probabilities between 0-100
char **edges; // stores unwieghted graph
mapData_t mapInfo;
MatrixXd mapSamples; // random samples in map
Vector3d X;
std::vector<pose_t> milestones;
pose_t wp1 = {4.0, 0.0, 0.0};
pose_t wp2 = {8.0, -4.0, 3.14};
pose_t wp3 = {8.0, 0.0, -1.57};

//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double x = msg.pose.pose.position.x; // Robot X psotition
	double y = msg.pose.pose.position.y; // Robot Y psotition
 	double yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	
	X << x, y, yaw;
}

//Example of drawing a curve
void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 

       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);

}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    //you probably want to save the map into a form which is easy to work with
	mapInfo.frame_id = msg.header.frame_id;
	mapInfo.res = msg.info.resolution;
	mapInfo.w = msg.info.width;
	mapInfo.h = msg.info.height;
	mapInfo.origin.x = msg.info.origin.position.x;
	mapInfo.origin.y = msg.info.origin.position.y;
 	mapInfo.origin.theta = tf::getYaw(msg.info.origin.orientation); // map's euler yaw
	occupancyGrid = (char**)malloc(mapInfo.h*sizeof(char*));
	for (auto i = 0; i < mapInfo.h; i++) {
		occupancyGrid[i] = (char*)malloc(mapInfo.w);
		memcpy(occupancyGrid[i], &msg.data[i*mapInfo.w], mapInfo.w);
	}
	mapRecieved = true;
}


void prm(ros::Rate &loop_rate) {

	// Waits for map to be recieved
	while (ros::ok() && !mapRecieved) {
		loop_rate.sleep();
		ros::spinOnce();
	}

	ROS_INFO_STREAM("Map Recieved\n");

	// Samples random configurations and stores collision free ones in milestones vector
	mapSamples = MatrixXd::Random(NUM_SAMPLES, 2 /* (x, y) samples */);
	milestones.push_back(pose_t {X(0), X(1), X(2)});
	milestones.push_back(wp1);
	milestones.push_back(wp2);
	milestones.push_back(wp3);
 	MatrixXf::Index maxRow, maxCol;
	double maxC = mapSamples.maxCoeff(&maxRow, &maxCol);
	for (auto i = 0; i < NUM_SAMPLES; i++) {
		//mapSamples(i,0) = fabs(mapSamples(i,0)) * mapInfo.h;
		mapSamples(i,0) = (mapSamples(i,0)/maxC) * mapInfo.h;
		//mapSamples(i,1) = fabs(mapSamples(i,1)) * mapInfo.w;
		mapSamples(i,1) = (mapSamples(i,1)/maxC) * mapInfo.w;
		//ROS_INFO_STREAM("xsample "<< mapSamples(i,0) << mapSamples(i,1) <<  "\n");
		int xSample = ((int)round(fabs(mapSamples(i,0))) >= mapInfo.h) ? mapInfo.h - 1 : (int)round(fabs(mapSamples(i,0)));
		int ySample = ((int)round(fabs(mapSamples(i,1))) >= mapInfo.w) ? mapInfo.w - 1 : (int)round(fabs(mapSamples(i,1)));
		if (occupancyGrid[xSample][ySample] < OCCUPIED_THRESHOLD) {
			milestones.push_back({mapSamples(i, 0) * mapInfo.res, mapSamples(i, 1) * mapInfo.res, 0});
		}
	}

	// - Find closest milestones
	// 		- Check if edge collides with any obstacles
	// 		- Only keep edges that are in collision free zones
	// 		- Put collision free edges in graph 2d array


	// Find shortest path from edges and milestones objects
}

void point_publisher() {
	visualization_msgs::Marker points;
	points.header.stamp = ros::Time::now();
	points.header.frame_id = "/map";
	points.ns = "turtle_points";
	points.action = visualization_msgs::Marker::ADD;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.pose.orientation.w = 1.0;
	points.scale.x = 0.2;
	points.scale.y = 0.2;
	points.color.g = 1.0f;
	points.color.a = 1.0;

	geometry_msgs::Point p;
	for (int j = 0; j < milestones.size(); j++) {
		p.x = milestones[j].x;
		p.y = milestones[j].y;
		p.z = 0;
		points.points.push_back(p);
	}
	marker_pub.publish(points);
}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
	
	prm(loop_rate);

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
		point_publisher();

		//Draw Curves
        drawCurve(1);
        drawCurve(2);
        drawCurve(4);
    
    	//Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed

    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
