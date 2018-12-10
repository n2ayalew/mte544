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

using namespace Eigen;

//Velocity control variable
geometry_msgs::Twist vel;

void carrot_controller(ros::Publisher velocity_publisher, int n) { //n is number of waypoints
  //Constants
  double Kp = 0.9; 
  double zeta = 2.0; 
  double L = 0.25; // threshold radius for waypoints 

  // Waypoint constants
  MatrixXd W(4,3);
  W(0,0) = X(0);
  W(0,1) = X(1);
  W(0,2) = X(2); 
  W(1,0) = 4.0; 
  W(1,1) = 0.0; 
  W(1,2) = 0.0;
  W(2,0) = 8.0; 
  W(2,1) = -4.0;
  W(2,2) = 3.14;
  W(3,0) = 8.0;
  W(3,1) = 0.0;
  W(3,2) = -1.57; 

  double lin_speed = 0.3; // linear speed m/s

  ros::spinOnce();
  //Loop through waypoints 
  for (int i = 0; i < n-1; i++) {
    double cur_waypt_x = W(i,0); // waypoint that robot has already passed
    double cur_waypt_y = W(i,1);
    double next_waypt_x = W(i+1,0); // waypoint that robot is travelling to 
    double next_waypt_y = W(i+1,1);

    // Continue until 
    while (!(fabs(X(0)-next_waypt_x) < L && fabs(X(1)-next_waypt_y) < L) ) {
      double ru = sqrt( pow(cur_waypt_x-X(0),2) + pow(cur_waypt_y-X(1),2)); // norm of vector 
      double ang = atan2(next_waypt_y-cur_waypt_y,next_waypt_x-cur_waypt_x); // angle between waypoints
      double angu = atan2(X(1)-cur_waypt_y, X(0)-cur_waypt_x); // angle between robot and waypoint that robot has passed 
      double beta = ang-angu; 

      double r = sqrt(pow(ru,2) - pow(ru*sin(beta),2)); 
      double x_prime = (r+zeta)*cos(ang) + cur_waypt_x; // x-coordinate of carrot 
      double y_prime = (r+zeta)*sin(ang) + cur_waypt_y; // y-coordinate of carrot 

      double steer_des = atan2(y_prime-X(1), x_prime-X(0));
      double error = steer_des - X(2); 

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
    ROS_INFO("Current X Position: %f", X(0));
    ROS_INFO("Current Y Position: %f", X(1));
    vel.linear.x = 0; 
    vel.angular.z = 0;
    velocity_publisher.publish(vel);
    ros::spinOnce();
  }
} 