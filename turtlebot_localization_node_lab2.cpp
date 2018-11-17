#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <ros/console.h>

//#include <boost/math/distributions/normal.hpp> // for normal_distribution
#include <boost/random.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <eigen3/Eigen/Dense>

#include <sstream>
#include <cstdint>
#include <time.h>
#include <math.h>
#include <random>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define NUM_PARTICLES 100
#define LOOP_RATE 20
#define MARKER_BUFFER_SIZE 10
#define FRAME_ID "/map"
#define SIM 1
#define MEASURMENT_NOISE 0.10
#define nts(n) n / 1000000000.0

using namespace Eigen;

ros::Publisher pose_publisher;
ros::Publisher marker_pub;
ros::Publisher path_pub;

double ips_x;
double ips_y;
double ips_yaw;

bool first_scan = true;
bool first_pose = true;

short sgn(int x) { return x >= 0 ? 1 : -1; }
double min(double a, double b) { return (a > b) ? b : a;}
double max(double a, double b) { return (a > b) ? a : b;}

// State Estimation Variables
Matrix<double, 3, NUM_PARTICLES> xbelief; 
MatrixXd xpredict;
Matrix<double, 1, NUM_PARTICLES> w;
Matrix<double, 1, NUM_PARTICLES> wsum;
Matrix3d rotation_matrix;
Matrix3d R;
Vector3d pose_estimate;
Vector3d xinital;
Matrix3d Q;
Matrix3d Qinv;
Vector3d u;
Vector3d y;
double dt_odom = 0.05;
//double w[NUM_PARTICLES] = {0};
//double wsum[NUM_PARTICLES] = {0};
std::random_device rd;  //Will be used to obtain a seed for the random number engine
std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
std::uniform_real_distribution<> dis(0.0, 1.0);

boost::variate_generator<boost::mt19937, boost::normal_distribution<double>>
							generator(boost::mt19937(time(0)), boost::normal_distribution<>(0.0, 1.0));

geometry_msgs::TwistWithCovariance odom_twist;
geometry_msgs::PoseWithCovariance odom_pose;

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg);
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
void odom_callback(const nav_msgs::Odometry &msg);
void state_prediction(double dt);
void state_belief_update();
void state_estimation(ros::Time t1);
void point_publisher(const ros::Time &t);
void robot_point_publisher(const ros::Time &t);
void pose_estimate_publisher(const ros::Time &t);
double mvnpdf(const Eigen::VectorXd &x, const Eigen::VectorXd &meanVec, const Eigen::MatrixXd &covMat);
double mvnpdf(const Eigen::VectorXd &Y, const Eigen::VectorXd &Xp);//, const Eigen::MatrixXd &covMat, const Eigen::MatrixXd &covMat);


int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_localization");
    ros::NodeHandle n;

	//std::random_device rd;
	//std::mt19937 gen;
	//gen.seed(rd());

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 5, pose_callback);

	// Subscribe to Odometry info
	ros::Subscriber odm = n.subscribe("/odom", 5, odom_callback);

    //Setup topics to Publish from this node
    //ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);

	// Particle Filter Publishing
    marker_pub = n.advertise<visualization_msgs::Marker>("/visualization_marker", 10, true);
	path_pub = n.advertise<nav_msgs::Path>("/path", 10, true);

	tf::Transform transform;
	tf::TransformBroadcaster br;

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(LOOP_RATE); //20Hz update rate

	xbelief = MatrixXd::Random(3, NUM_PARTICLES);
	xpredict = xbelief;//MatrixXd::Zero(3, NUM_PARTICLES);
	//w = RowVectorXd::Zero(NUM_PARTICLES);
	//wsum = RowVectorXd::Zero(NUM_PARTICLES);
	//std::random_device rd{};
	//std::mt19937 gen{rd()};
	//std::normal_distribution<double> nde{0.0, 1.0};
	/*for (auto i = 0; i < NUM_PARTICLES; i++) {
		xbelief.col(i) *= generator();
	}*/
	auto t1 = ros::Time::now();
	while (ros::ok()) {
        ros::spinOnce();
        // vel.linear.x = 0;
        // vel.angular.z = 0.5; // set angular speed
        // velocity_publisher.publish(vel); // Publish the command velocity
		state_estimation(t1);//, gen, nde);
		t1 = ros::Time::now();
		pose_estimate_publisher(t1);
		point_publisher(t1);
        loop_rate.sleep(); //Maintain the loop rate
    }
    return 0;
}

void pose_estimate_publisher(const ros::Time &t) {
	geometry_msgs::PoseStamped pose;
	nav_msgs::Path path;
	path.header.stamp = t;
	path.header.frame_id = FRAME_ID;
	pose.header.stamp = t;
	pose.header.frame_id = FRAME_ID;
	pose.pose.position.x = pose_estimate(0);
	pose.pose.position.y = pose_estimate(1);
	pose.pose.orientation.z = std::sin(pose_estimate(2)/2);
	pose.pose.orientation.w = std::cos(pose_estimate(2)/2);
	path.poses.push_back(pose);
	pose_publisher.publish(pose);	
	path_pub.publish(path);
}

void point_publisher(const ros::Time &t) {
	visualization_msgs::Marker points;
	points.header.stamp = t;
	points.header.frame_id = FRAME_ID;
	points.ns = "turtle_points";
	points.action = visualization_msgs::Marker::ADD;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.pose.orientation.w = 1.0;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.g = 1.0f;
	points.color.a = 1.0;

	geometry_msgs::Point p;
	for (int j = 0; j < NUM_PARTICLES; j++) {
		p.x = xbelief(0,j);
		p.y = xbelief(1,j);
		p.z = 0;
		points.points.push_back(p);
	}

//	ROS_INFO_STREAM("x is " << xbelief(0,50) << " and y is " << xbelief(1,50) );
	marker_pub.publish(points);
}

// down sample pose: rosrun throttle messages gazebo/model_states 1.0
void robot_point_publisher(const ros::Time &t) {
	visualization_msgs::Marker points;
	points.header.stamp = t;
	points.header.frame_id = FRAME_ID;
	points.ns = "bot_points";
	points.action = visualization_msgs::Marker::ADD;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.pose.orientation.w = 1.0;
	points.scale.x = 0.3;
	points.scale.y = 0.3;
	points.color.r = 1.0f;
	points.color.a = 1.0;

	geometry_msgs::Point p;
	p.x = ips_x;
	p.y = ips_y;
	p.z = 0;
	points.points.push_back(p);
	marker_pub.publish(points);
}
void pose_callback(const gazebo_msgs::ModelStates &msg) {
	int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;
	
	ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
	/*if(ips_yaw<0) {
		ips_yaw = ips_yaw + 2*M_PI;
	}*/
	y << ips_x, ips_y, ips_yaw;

	if (first_pose) {
		first_pose = false;
		xbelief.row(0) = ((xbelief.row(0).array()) + ips_x).matrix();
		xbelief.row(1) = ((xbelief.row(1).array()) + ips_y).matrix();
		xbelief.row(2) = ((xbelief.row(2).array()) + ips_yaw).matrix();
		xpredict = xbelief;
	}
	auto t1 = ros::Time::now();
	robot_point_publisher(t1);
}


void odom_callback(const nav_msgs::Odometry &msg) {
	static double last_odom_time;
	odom_pose = msg.pose;
	odom_twist = msg.twist;
	R << odom_pose.covariance[0], 0, 0, 0,  odom_pose.covariance[7], 0, 0, 0, odom_pose.covariance[35];
	u << odom_twist.twist.linear.x, odom_twist.twist.linear.y, odom_twist.twist.angular.z;
	//dt_odom = nts(msg.header.stamp.nsec - last_odom_time);
	last_odom_time = msg.header.stamp.nsec;
}

// down sample pose: rosrun throttle messages pose 1.0
//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	ips_x = msg.pose.pose.position.x; // Robot X psotition
	ips_y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	/*if(ips_yaw<0) {
		ips_yaw = ips_yaw + 2*M_PI;
	}*/
	//Q << msp.covariance[0], 0, 0, 0, msp.covariance[8], 0, 0, 0, msp.covariance[35];
	y << ips_x, ips_y, ips_yaw;
	if (first_pose) {
		first_pose = false;
		xbelief.row(0) = ((xbelief.row(0).array()) + ips_x).matrix();
		xbelief.row(1) = ((xbelief.row(1).array()) + ips_y).matrix();
		xbelief.row(2) = ((xbelief.row(2).array()) + ips_yaw).matrix();
		xpredict = xbelief;
	}
}

void state_prediction(double dt){
	const double meas_var = 0.01;//MEASURMENT_NOISE*MEASURMENT_NOISE;
	Vector3d randVector(0,0,0);
	//Vector3d e;
	randVector << generator() , generator() , generator();
#ifdef SIM
	Q << meas_var, 0, 0, 0, meas_var, 0, 0, 0, meas_var;
	Qinv << 1/meas_var, 0, 0, 0, 1/meas_var, 0, 0, 0, 1/meas_var;
#endif
	//y << ips_x + d(0), ips_y + d(1), ips_yaw + d(2);
	Vector3d d = Q*randVector;
	y = y + d;
	for (auto m = 0; m < NUM_PARTICLES; m++) {
		randVector << generator() , generator() , generator();
		
		//rotation_matrix << std::cos(xbelief(2,m)), -std::sin(xbelief(2,m)), 0, std::sin(xbelief(2,m)), std::cos(xbelief(2,m)), 0, 0, 0, 1; // body to inertial F.O.R 
		rotation_matrix << std::cos(xbelief(2,m)), 0, 0, 0, std::sin(xbelief(2,m)), 0, 0, 0, 1; // body to inertial F.O.R
		xpredict.col(m) = xbelief.col(m) + rotation_matrix*(u*dt_odom) + R*randVector;
		if (xpredict(2,m) > M_PI) {
			xpredict(2,m) += -2*M_PI;
		}
		if (xpredict(2,m) < -M_PI) {
			xpredict(2,m) += 2*M_PI;
		}

		w(m) = mvnpdf(y, xpredict.col(m), Q); 
		//ROS_INFO("w(m) = %f", w(m));
		//ROS_INFO("w(m) = %f, xpredict(m) = (%f, %f, %f), y = (%f, %f, %f)\n", w(m), xpredict(0,m), xpredict(1,m), xpredict(2,m), y(0), y(1), y(2));
		if (m > 0 ) {
			wsum(m) = w(m) + wsum(m-1);
		} else {
			wsum(m) = w(m);
		} 
	}
}

void state_belief_update(){
	for (auto m = 0; m < NUM_PARTICLES; m++) {
		//MatrixXd::Index maxCol;
		//auto seed = wsum.maxCoeff(&maxCol)*generator();
		auto seed = wsum(NUM_PARTICLES-1)*dis(gen);
		//ROS_INFO("seed = %f\n", seed);
		for (auto i = 0; i < NUM_PARTICLES; i++){
			if (wsum(i) > seed){ 
				xbelief.col(m) = xpredict.col(i);
				break;
			}
		}
	}
	 
}

void state_estimation(ros::Time t1){
	auto dt = (ros::Time::now() - t1).toSec();	
	state_prediction(dt);
	state_belief_update();
	
	pose_estimate(0) = xbelief.row(0).mean();
	pose_estimate(1) = xbelief.row(1).mean();
	pose_estimate(2) = xbelief.row(2).mean();
	//ROS_INFO("pose_est(m) = (%f, %f, %f), y = (%f, %f, %f)\n", pose_estimate(0), pose_estimate(1), pose_estimate(2), y(0), y(1), y(2));
	//pose_var(0) = xbelief.row(0).mean();
	//pose_var(1) = xbelief.row(1).mean();
	//pose_var(2) = xbelief.row(2).mean();
}


 // Taken out b/c inefficent
double mvnpdf(const VectorXd &x, const VectorXd &meanVec, const MatrixXd &covMat)
{
	// avoid magic numbers in your code. Compilers will be able to compute this at compile time:
	const double logSqrt2Pi = 0.5*std::log(2*M_PI);
	typedef LLT<MatrixXd> Chol;
	Chol chol(covMat);
	// Handle non positive definite covariance somehow:
	if(chol.info()!= Success) throw "decomposition failed!";
	const Chol::Traits::MatrixL& L = chol.matrixL();
	double quadform = (L.solve(x - meanVec)).squaredNorm();
	//ROS_INFO("quadform = %f, L.determinant = %f\n", quadform, L.determinant());
	return std::exp(-x.rows()*logSqrt2Pi - 0.5*quadform) / L.determinant();
}

/*
double mvnpdf(const Eigen::VectorXd &Y, const Eigen::VectorXd &Xp){// , const Eigen::MatrixXd &covMat, const Eigen::MatrixXd &covMat) {
	//const double q_det = std::pow(std::pow(MEASURMENT_NOISE,2)*2*M_PI, 3);
	const double q_det = std::pow(MEASURMENT_NOISE*MEASURMENT_NOISE*2*M_PI, 3);
	const double normalizer = std::pow(q_det, -0.5);
	double quadform = ((Y - Xp).transpose() * Qinv) * (Y - Xp);
	//ROS_INFO("quadform = %f, q_det = %f, normalizer = %f\n", quadform, q_det, normalizer);
	//ROS_INFO("q_det = %f, normalizer = %f\n", q_det, normalizer);
	return normalizer * std::exp(-0.5*quadform);
}
*/
