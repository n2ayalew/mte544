//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It publishes a pose of the robot in simulation with respect 
// to the inertial frame.
//
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

// This executable allows us to subscribe to indoor_pos by acting
// as an intermeditary between gazebo and indoor_pos.
// Takes care of any nessecary transforms

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher pose_publisher;
tf::TransformBroadcaster *br;
tf::Transform *tform;

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates& msg)
{

	int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

	//This function is called when a new position message is received
	geometry_msgs::PoseWithCovarianceStamped curpose;
	curpose.header.stamp = ros::Time::now();
	curpose.header.frame_id="/map";
	curpose.pose.pose.position = msg.pose[i].position;
	curpose.pose.pose.orientation = msg.pose[i].orientation;
	pose_publisher.publish(curpose);

	// send transform
	br = new tf::TransformBroadcaster;
	tform = new tf::Transform;
	tform->setOrigin( tf::Vector3(msg.pose[i].position.x, msg.pose[i].position.y, 0) );
	tf::Quaternion q;
	q.setEulerZYX(tf::getYaw(msg.pose[i].orientation), 0, 0);
	tform->setRotation( q );
	*tform = tform->inverse();
	br->sendTransform(tf::StampedTransform(*tform, ros::Time::now(), "base_footprint", "map"));
}


int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"sim_pose_publisher");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    pose_publisher = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/indoor_pos", 1, true);
  

    //Set the loop rate
    ros::Rate loop_rate(40);    //40Hz update rate
	

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages
    }

    return 0;
}
