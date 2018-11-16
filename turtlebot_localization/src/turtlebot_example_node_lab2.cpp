//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <eigen3/Eigen/Dense>
#include <std_msgs/String.h>

#include <boost/shared_ptr.hpp>
#include <sstream>
#include <cstdint>
#include <math.h>
#include <random>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define M 300
#define N 300
#define RESOLUTION 0.11
#define NUM_PARTICLES 100
#define LOOP_RATE 20
//#define MAPPING 1
using namespace Eigen;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

bool first_scan = true;
bool first_pose = true;
sensor_msgs::LaserScan scanner_msg;

short sgn(int x) { return x >= 0 ? 1 : -1; }
double min(double a, double b) { return (a > b) ? b : a;}
double max(double a, double b) { return (a > b) ? a : b;}

//Consts used for mapping
float angle_min = 0;
float angle_max = 0;
float angle_increment = 0;
float range_min = 0;
float range_max = 0;
int numRanges = 0;
MatrixXd belief_map;
nav_msgs::OccupancyGrid map_msg;

// State Estimation
MatrixXd xbelief; 
MatrixXd xpredict;
Vector3d xinital;
geometry_msgs::TwistWithCovariance odom_twist;
geometry_msgs::PoseWithCovariance odom_pose;

void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y);

// Calculates the inverse measurement model for a laser scanner through
// raytracing with Bresenham's algorithm, assigns low probability of object
// on ray, high probability at end.  Returns cells and probabilities.
void inversescannerbres(Vector3d &x, float phi, float range, std::vector<int> &linex, std::vector<int> &liney, std::vector<float> &linepm){//MatrixXf &invmod) {
	//if (range < range_min || range > range_max) return;
	if (std::isnan(range)) return;
	auto angle = phi + x(2);
	// Range finder inverse measurement model
	// int x1 = max(0.0, min((double)M, x(0)/RESOLUTION));
	int x1 = max(0.0, min((double)M-1, x(0)/RESOLUTION+(M/2.0))); //Adding M/2 as an offset since the bottom left-hand corner is 0,0
	int y1 = max(0.0, min((double)N-1, x(1)/RESOLUTION+(N/2.0))); //Adding M/2 as an offset since the bottom left-hand corner is 0,0
	auto endptx = (x(0) + range*cos(angle))/RESOLUTION;
	auto endpty = (x(1) + range*sin(angle))/RESOLUTION;

	int x2 = max(0.0, min((double)M-1, endptx+(M/2.0))); //Adding M/2 as an offset since the bottom left-hand corner is 0,0
	int y2 = max(0.0, min((double)N-1, endpty+(N/2.0))); //Adding M/2 as an offset since the bottom left-hand corner is 0,0

	bresenham(x1,y1,x2,y2, linex, liney);
	//invmod.resize(linex.size(), 3);
	for (auto i = 0; i < linex.size(); i++) {
		//invmod(i, 0) = linex[i];
		//invmod(i, 1) = liney[i];
		//invmod(i, 2) = 0.4;
		linepm.push_back(0.4);
	//	ROS_INFO("ix = %f, iy = %f, range = %f, size = %lu\n", invmod(i,0),invmod(i,1), range, linex.size());
	}
	if (range < range_max){
		linepm[linex.size() - 1] = 0.6;
	}
}

void map_update(Vector3d &x) {
	for (auto i = 0; i < numRanges; i++) {
		// // Get inverse measurement model
		// if (!(scanner_msg.ranges[i] < range_max && scanner_msg.ranges[i] > range_min)) {
		// 	//ROS_INFO("out of range (%f)\n", scanner_msg.ranges[i]);
		// 	continue;
		// }

		if (std::isnan(scanner_msg.ranges[i])) {
			continue;
		}
		/*if (scanner_msg.ranges[i] < range_min) {
			ROS_INFO("range to small = %f\n", scanner_msg.ranges[i]);
			continue;
		}*/
		MatrixXf invmod(numRanges, 3);
		std::vector<int> linex;
		std::vector<int> liney;
		std::vector<float> linepm;
		auto cur_phi = angle_min + (i * angle_increment);
		inversescannerbres(x, cur_phi, scanner_msg.ranges[i], linex, liney, linepm);//, invmod);
		for (auto j = 0; j < linex.size(); j++) { //:length(invmod(:,1)) {
			int ix = linex[j];//invmod(j,0);
			int iy = liney[j];//invmod(j,1);
			float il = linepm[j];//invmod(j,2);
			//ROS_INFO("ix = %f, iy = %f, range = %f\n", ix, iy, scanner_msg.ranges[i]);
			// Calculate updated log odds
			belief_map(ix,iy) = belief_map(ix,iy) + log(il/(1-il)); // - L0(ix,iy);
			//ROS_INFO("Belief Map[%d, %d]: %f\n", ix, iy, belief_map(ix,iy)); 
		}
	}
}

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {
    static int ips_update_counter = 0; // Only used for simulating positon
	int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;
	ips_update_counter++;
	if (ips_update_counter % LOOP_RATE != 0) return; // only update ips at ~ 1Hz
	ips_update_counter = 0;
	ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
	//if (first_pose) {
		//map_msg.info.origin.orientation = msg.pose[i].orientation;
	//first_pose = false;
	xbelief.row(0) = ((xbelief.row(0).array()*5) + ips_x).matrix();
	xbelief.row(1) = ((xbelief.row(1).arra()*5) + ips_y).matrix();
	xbelief.row(2) = ((xbelief.row(2).array()*M_PI) + ips_yaw).matrix();
	//}
}

void scan_callback(const sensor_msgs::LaserScan &msg) {
	//if (first_scan) {
		angle_min = msg.angle_min;
		angle_max = msg.angle_max;
		angle_increment = msg.angle_increment;
		range_min = msg.range_min;
		range_max = msg.range_max;      	
		numRanges = (angle_max-angle_min)/angle_increment;
		// numRanges = sizeof(msg.ranges)/4;
		first_scan = false;
		//ROS_INFO("angle_min = %f, angle_max = %f, range_min = %f, range_max = %f\n", angle_min, angle_max, range_min, range_max);
	//} else {
		scanner_msg.ranges = msg.ranges;
	//}
}

//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

void odom_callback(const nav_msgs::Odometery &msg) {
	odom_pose = msg.pose;
	odom_twist = msg.twist;
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}

void prediction_state(double dt) {
	Vector3d u(odm_twist.linear.x, odm_twist.linear.y, odm_twist.angular.x);
	Vector3d e(0.1, 0.1, 0);
	std::random_device rd{};
	std::mt19937 gen{rd};
	for (auto m = 0; m < NUM_PARTICLES; m++) {
		//e = sqrt(R)*randn(1);
		xpredict.col(m) = xbelief.col(m) + u + e;
		//w(m) = normpdf(y(t),Xp(m),Q);

	}

}
/*
void state_belief_update() {
	for (auto i = 0; i < NUM_PARTICLES; i++) {
		seed = W(end)*rand(1);
		X(m) = Xp(find(W>seed,1));

	}
}
*/
void state_estimation(ros::Rate &loop_rate) {
	auto t1 = ros::Time::now();
	while (ros::ok()) {
		ros::spinOnce();
		auto dt = ros::Time::now() - t1;	
		state_prediction(dt);
		t1 = ros::Time::now();
//		W = cumsum(w);
		//xinital << ips_x, ips_y, ips_yaw;
		loop_rate.sleep();
	}	
}

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_mapping");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);

	// Subscribe to laser scans
	ros::Subscriber laser_sub = n.subscribe("/scan", 1, scan_callback);

	// Subscribe to Odometery info
	ros::Subscribe odm = n.subscribe("/odom", 1, odom_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
	ros::Publisher map_publisher = n.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

	tf::Transform transform;
	tf::TransformBroadcaster br;

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(LOOP_RATE); //20Hz update rate

	ros::spinOnce(); // Get inital position, and other constants
	//ROS_INFO("laser min angle: %f, max angle: %f, timestamp: %u\n" , scanner_msg.angle_min, scanner_msg.angle_max, scanner_msg.header.seq);  
    
	Vector3d X0(ips_x, ips_y, ips_yaw); 
	Vector3d X(ips_x, ips_y, ips_yaw); 
	belief_map = MatrixXd::Zero(M,N);
	std::vector<signed char> occupancy_grid(M*N);// = MatrixXd::Zero(M,N);
	
	map_msg.data.clear();
	map_msg.header.frame_id = "map";
	map_msg.info.map_load_time = ros::Time::now(); 
	// map_msg.info.origin.position.x = X(0);
	// map_msg.info.origin.position.y = X(1);
	// map_msg.info.origin.position.z = X(2);
	// ROS_INFO("Initial Positions x: %f, y: %f, z: %f", X(0), X(1), X(2));
	geometry_msgs::Pose origin;
	// map_msg.info.origin.position.x = -(M*RESOLUTION)/2.0;
	// map_msg.info.origin.position.y = -(N*RESOLUTION)/2.0;
	// map_msg.info.origin.position.z = 0;
	map_msg.info.width = N;
	map_msg.info.height = M;
	map_msg.info.resolution = RESOLUTION;

	origin.position.x = -(M*RESOLUTION)/2.0;
	origin.position.y = -(N*RESOLUTION)/2.0;
	origin.position.z = 0; 

	origin.orientation.x = 0;
	origin.orientation.y = 0;
	origin.orientation.z = 0;
	map_msg.info.origin = origin;
	map_msg.data.resize(M*N);
	map_publisher.publish(map_msg);

	xbelief = MatrixXd::Random(3, NUM_PARTICLES);
	xperdict = MatrixXd::Zeros(3, NUM_PARTICLES);

	while (ros::ok()) {
        //Main loop code goes here:
        // vel.linear.x = 0;
        // vel.angular.z = 0.5; // set angular speed
        // velocity_publisher.publish(vel); // Publish the command velocity
        ros::spinOnce();   //Check for new messages
		X << ips_x, ips_y, ips_yaw; 

		//ROS_INFO("angle_min = %f, angle_max = %f, range_min = %f, range_max = %f\n", angle_min, angle_max, range_min, range_max);
		map_update(X);
		//ROS_INFO("x = %f, y = %f, theta = %f\n" , X(0), X(1), X(2));  
		for (auto i = 0; i < M; i++){
			for (auto j = 0; j < N; j++) {
				occupancy_grid[i*N+j] = round( (exp(belief_map(i, j)) / (1.0 + exp(belief_map(i, j)))) * 100.0);
				//ROS_INFO("Belief Map[%d, %d]: %f, ", i, j, belief_map(i,j)); 
				// ROS_INFO("map[%d]: %d, " , i*N+j, occupancy_grid[i*N+j]);  
			}
		}
		map_msg.data = occupancy_grid;
	//	ROS_INFO("map_msg[%d]: %d\n" , M*N-1, map_msg.data[M*N-1]);  
		map_msg.info.map_load_time = ros::Time::now(); 
		map_publisher.publish(map_msg);	
        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}