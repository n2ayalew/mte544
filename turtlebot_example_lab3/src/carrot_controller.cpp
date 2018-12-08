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
#include <cmath>

ros::Publisher marker_pub;

using namespace Eigen;

#define TAGID 0

double ips_x; 
double ips_y; 
double ips_yaw; 

// Carrot Controller Constants


    //Velocity control variable
geometry_msgs::Twist vel;


//Callback function for the Position topic (LIVE)
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
  ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
}

// //Callback function for the Position topic (SimplyMULATION)
// void pose_callback(const gazebo_msgs::ModelStates &msg) {
//   int i;
//   for (i = 0; i < msg.name.size(); i++) {
//     if (msg.name[i] == "mobile_base")
//       break;
//   }
//   ips_x = msg.pose[i].position.x;
//   ips_y = msg.pose[i].position.y;
//   ips_yaw = tf::getYaw(msg.pose[i].orientation);
// }

void carrot_controller(ros::Publisher velocity_publisher, int n, MatrixXd W) {
  //Constants
  double Kp = 0.9; 
  double zeta = 2.0; 
  double L = 0.25; // threshold radius for waypoints 

  double lin_speed = 0.3; // linear speed m/s

  ros::spinOnce();
  //Loop through waypoints 
  for (int i = 0; i < n-1; i++) {
    double cur_waypt_x = W(i,0); // waypoint that robot has already passed
    double cur_waypt_y = W(i,1);
    double next_waypt_x = W(i+1,0); // waypoint that robot is travelling to 
    double next_waypt_y = W(i+1,1);

    // Continue until 
    while (!(fabs(ips_x-next_waypt_x) < L && fabs(ips_y-next_waypt_y) < L) ) {
      double ru = sqrt( pow(cur_waypt_x-ips_x,2) + pow(cur_waypt_y-ips_y,2)); // norm of vector 
      double ang = atan2(next_waypt_y-cur_waypt_y,next_waypt_x-cur_waypt_x); // angle between waypoints
      double angu = atan2(ips_y-cur_waypt_y, ips_x-cur_waypt_x); // angle between robot and waypoint that robot has passed 
      double beta = ang-angu; 

      double r = sqrt(pow(ru,2) - pow(ru*sin(beta),2)); 
      double x_prime = (r+zeta)*cos(ang) + cur_waypt_x; // x-coordinate of carrot 
      double y_prime = (r+zeta)*sin(ang) + cur_waypt_y; // y-coordinate of carrot 

      double steer_des = atan2(y_prime-ips_y, x_prime-ips_x);
      double error = steer_des - ips_yaw; 

      if (error < -M_PI)
        error += 2*M_PI; 
      else if (error > M_PI)
        error -= 2*M_PI; 

      double u = Kp*error;
      vel.linear.x = lin_speed; 
      vel.angular.z = u; 

      velocity_publisher.publish(vel);

      ros::spinOnce(); 
    }
    ROS_INFO("Current X Position: %f", ips_x);
    ROS_INFO("Current Y Position: %f", ips_y);
    vel.linear.x = 0; 
    vel.angular.z = 0;
    velocity_publisher.publish(vel);
    ros::spinOnce();
  }
} 

// //Example of drawing a curve
// void drawCurve(int k) 
// {
//    // Curves are drawn as a series of stright lines
//    // Simply sample your curves into a series of points

//    double x = 0;
//    double y = 0;
//    double steps = 50;

//    visualization_msgs::Marker lines;
//    lines.header.frame_id = "/map";
//    lines.id = k; //each curve must have a unique id or you will overwrite an old ones
//    lines.type = visualization_msgs::Marker::LINE_STRIP;
//    lines.action = visualization_msgs::Marker::ADD;
//    lines.ns = "curves";
//    lines.scale.x = 0.1;
//    lines.color.r = 1.0;
//    lines.color.b = 0.2*k;
//    lines.color.a = 1.0;

//    //generate curve points
//    for(int i = 0; i < steps; i++) {
//        geometry_msgs::Point p;
//        p.x = x;
//        p.y = y;
//        p.z = 0; //not used
//        lines.points.push_back(p); 

//        //curve model
//        x = x+0.1;
//        y = sin(0.1*i*k);   
//    }

//    //publish new curve
//    marker_pub.publish(lines);

// }

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
    // ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
      
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    ros::spinOnce();
    ROS_INFO("ips_x: %f", ips_x);
    ROS_INFO("ips_y: %f", ips_y); 

    // Testing purposes only, remove later
    MatrixXd W(4,2); 
    W(0,0) = ips_x;
    W(0,1) = ips_y; 
    W(1,0) = 4.0; 
    W(1,1) = 0.0; 
    W(2,0) = 8.0;
    W(2,1) = -4.0;
    W(3,0) = 8.0; 
    W(3,1) = 0.0;

	  int num = W.rows();
    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    	//Main loop code goes here:
      carrot_controller(velocity_publisher, num, W);
      break; // Program should end after carrot_controller is done working
    }

    return 0;
}
