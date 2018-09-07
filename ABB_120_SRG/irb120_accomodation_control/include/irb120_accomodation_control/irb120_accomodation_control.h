#ifndef BIN_INVENTORY_H_
#define BIN_INVENTORY_H_
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <irb120_accomodation_control/irb120_accomodation_control.h>

using namespace std;

class Irb120AccomodationControl {
	public:
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist);
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist, Eigen::MatrixXf accomodation_gain);
	void findJointVelFromCartVel (geometry_msgs::Twist twist,sensor_msgs::JointState, Eigen::MatrixXf jacobian, vector<float> &joint_vel);
	void findJointVelFromCartVel (geometry_msgs::Twist twist, sensor_msgs::JointState, vector<float> &joint_vel);
	void publishJointAngles(vector<float> joint_pos);
	void publishJointAngles(vector<std_msgs::Float64> joint_pos);
	void jointStateCallBack (const sensor_msgs::JointState &joint_state); 
	void ftCallBack (const geometry_msgs::WrenchStamped &wrench_stamped);
	sensor_msgs::JointState getJointState();
	geometry_msgs::Wrench getFTSensorValue();
	void commandJointPosFromJointVel(vector<float> joint_vel, sensor_msgs::JointState joint);
	Irb120AccomodationControl(ros::NodeHandle &nh);
	Eigen::MatrixXf getJacobian();


	private:
	void warmUp();
	void initializeJacobian(sensor_msgs::JointState joint_states);
	float dt = 0.01;
	sensor_msgs::JointState g_joint_state;
	geometry_msgs::Wrench g_ft_value;
	Eigen::MatrixXf accomodation_gain = Eigen::MatrixXf::Identity(6,6);
	Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(6,6); //Need to initialize this
	Eigen::MatrixXf jacobian_inverse = jacobian.inverse();
	const string joint1_topic_name = "/irb120/joint1_position_controller/command";
	const string joint2_topic_name = "/irb120/joint2_position_controller/command";
	const string joint3_topic_name = "/irb120/joint3_position_controller/command";
	const string joint4_topic_name = "/irb120/joint4_position_controller/command";
	const string joint5_topic_name = "/irb120/joint5_position_controller/command";
	const string joint6_topic_name = "/irb120/joint6_position_controller/command";
	const string joint_state_subscriber_topic = "/irb120/joint_states";
	const string ft_value_subscriber_topic = "/ft_sensor_topic_fake";
	ros::Publisher joint1_pub, joint2_pub, joint3_pub, joint4_pub, joint5_pub, joint6_pub;
	ros::Subscriber joint_state_subscriber, ft_value_subscriber;
	

};
#endif