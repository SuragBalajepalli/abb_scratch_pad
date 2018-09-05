/*#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

using namespace std;

class Irb120AccomodationControl {
	public:
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist);
	void findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist, Eigen::MatrixXf accomodation_gain);
	void findJointVelFromCartVel (geometry_msgs::Twist twist, Eigen::MatrixXf jacobian, vector<float> &joint_vel);
	void findJointVelFromCartVel (geometry_msgs::Twist twist, vector<float> &joint_vel);
	void publishJointAngles(vector<float> joint_pos);
	void publishJointAngles(vector<std_msgs::Float64> joint_pos);
	void jointStateCallBack (const sensor_msgs::JointState &joint_state); 
	void ftCallBack (const geometry_msgs::WrenchStamped &wrench_stamped);
	sensor_msgs::JointState getJointState();
	geometry_msgs::Wrench getFTSensorValue();
	Irb120AccomodationControl(ros::NodeHandle &nh);

	private:
	geometry_msgs::Wrench g_ft_value;
	sensor_msgs::JointState g_joint_state;
	const Eigen::MatrixXf accomodation_gain = Eigen::MatrixXf::Identity(6,6);
	const Eigen::MatrixXf jacobian = Eigen::MatrixXf::Zero(6,6); //Need to initialize this
	const Eigen::MatrixXf jacobian_inverse = jacobian.inverse();
	const string joint1_topic_name = "/irb120/joint1_position_controller/command";
	const string joint2_topic_name = "/irb120/joint2_position_controller/command";
	const string joint3_topic_name = "/irb120/joint3_position_controller/command";
	const string joint4_topic_name = "/irb120/joint4_position_controller/command";
	const string joint5_topic_name = "/irb120/joint5_position_controller/command";
	const string joint6_topic_name = "/irb120/joint6_position_controller/command";
	const string joint_state_subscriber_topic = "/irb120/joint_states";
	const string ft_value_subscriber_topic = "/ft_sensor_topic";
	ros::Publisher joint1_pub, joint2_pub, joint3_pub, joint4_pub, joint5_pub, joint6_pub;
	ros::Subscriber joint_state_subscriber, ft_value_subscriber;
	

}; */

#include <irb120_accomodation_control/irb120_accomodation_control.h>
	Irb120AccomodationControl::Irb120AccomodationControl(ros::NodeHandle &nh) {
		
		//setting up all publishers
		joint1_pub = nh.advertise<std_msgs::Float64>(joint1_topic_name, 1);
		joint2_pub = nh.advertise<std_msgs::Float64>(joint2_topic_name, 1);
		joint3_pub = nh.advertise<std_msgs::Float64>(joint3_topic_name, 1);
		joint4_pub = nh.advertise<std_msgs::Float64>(joint4_topic_name, 1);
		joint5_pub = nh.advertise<std_msgs::Float64>(joint5_topic_name, 1);
		joint6_pub = nh.advertise<std_msgs::Float64>(joint6_topic_name, 1);
		
		//setting up all subscribers
		joint_state_subscriber = nh.subscribe(joint_state_subscriber_topic, 1, &Irb120AccomodationControl::jointStateCallBack, this);
		ft_value_subscriber = nh.subscribe(ft_value_subscriber_topic,1,&Irb120AccomodationControl::ftCallBack, this);

	}

	void Irb120AccomodationControl::findCartVelFromWrench(geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist) {
		//uses accomodation gain described in class
		Eigen::VectorXf wrench_matrix(6); //since operations need to be performed
		Eigen::VectorXf twist_matrix(6);
		wrench_matrix<<wrench.force.x, 
						wrench.force.y,
						wrench.force.z,
						wrench.torque.x,
						wrench.torque.y,
						wrench.torque.z; 
		twist_matrix = accomodation_gain * wrench_matrix; //maybe the other way round, check it out, makes no difference with I
		twist.linear.x = twist_matrix(0); //rethink the need to populate this message
		twist.linear.y = twist_matrix(1);
		twist.linear.z = twist_matrix(2);
		twist.angular.x = twist_matrix(3);
		twist.angular.y = twist_matrix(4);
		twist.angular.z = twist_matrix(5);
	}

	void Irb120AccomodationControl::findCartVelFromWrench (geometry_msgs::Wrench wrench, geometry_msgs::Twist &twist,
														 Eigen::MatrixXf given_accomodation_gain) {
		//for user defined accomodation gain
		Eigen::VectorXf wrench_matrix(6); //since operations need to be performed
		Eigen::VectorXf twist_matrix(6);
		wrench_matrix<<wrench.force.x, 
						wrench.force.y,
						wrench.force.z,
						wrench.torque.x,
						wrench.torque.y,
						wrench.torque.z; 
		twist_matrix = wrench_matrix * given_accomodation_gain; //maybe the other way round, check it out, makes no difference with I
		twist.linear.x = twist_matrix(0); //rethink the need to populate this message
		twist.linear.y = twist_matrix(1);
		twist.linear.z = twist_matrix(2);
		twist.angular.x = twist_matrix(3);
		twist.angular.y = twist_matrix(4);
		twist.angular.z = twist_matrix(5);	
	}

	void Irb120AccomodationControl::findJointVelFromCartVel (geometry_msgs::Twist twist, Eigen::MatrixXf given_jacobian, vector<float> &joint_vel) {
		//takes cart vel input in twist form, returns a 6 element vector of joint vel
		//User defined jacobian
		joint_vel.clear();
		Eigen::VectorXf twist_matrix(6);
		Eigen::VectorXf joint_velocities(6);
		twist_matrix<<twist.linear.x,
						twist.linear.y,
						twist.linear.z,
						twist.angular.x,
						twist.angular.y,
						twist.angular.z;
		joint_velocities = given_jacobian.inverse() * twist_matrix;
		joint_vel.push_back(joint_velocities(0));
		joint_vel.push_back(joint_velocities(1));
		joint_vel.push_back(joint_velocities(2));
		joint_vel.push_back(joint_velocities(3));
		joint_vel.push_back(joint_velocities(4));
		joint_vel.push_back(joint_velocities(5));
	}

	void Irb120AccomodationControl::findJointVelFromCartVel (geometry_msgs::Twist twist, vector<float> &joint_vel) {
		//same as above, except
		//uses predefined jacobian
		joint_vel.clear();
		Eigen::VectorXf twist_matrix(6);
		Eigen::VectorXf joint_velocities(6);
		twist_matrix<<twist.linear.x,
						twist.linear.y,
						twist.linear.z,
						twist.angular.x,
						twist.angular.y,
						twist.angular.z;
		joint_velocities = jacobian_inverse * twist_matrix;
		joint_vel.push_back(joint_velocities(0));
		joint_vel.push_back(joint_velocities(1));
		joint_vel.push_back(joint_velocities(2));
		joint_vel.push_back(joint_velocities(3));
		joint_vel.push_back(joint_velocities(4));
		joint_vel.push_back(joint_velocities(5));
	}

	void Irb120AccomodationControl::publishJointAngles(vector<std_msgs::Float64> joint_pos) {
		joint1_pub.publish(joint_pos[0]);
		joint2_pub.publish(joint_pos[1]);
		joint3_pub.publish(joint_pos[2]);
		joint4_pub.publish(joint_pos[3]);
		joint5_pub.publish(joint_pos[4]);
		joint6_pub.publish(joint_pos[5]);
	}

	void Irb120AccomodationControl::publishJointAngles(vector<float> joint_pos) {
		vector<std_msgs::Float64> joint_pos_msg(6);
		for(int i = 0; i < joint_pos.size(); i++) {
			joint_pos_msg[i].data = joint_pos[i];
		} 
		publishJointAngles(joint_pos_msg);

	}

	void Irb120AccomodationControl::jointStateCallBack(const sensor_msgs::JointState &joint_state) {
		g_joint_state = joint_state;
	}

	void Irb120AccomodationControl::ftCallBack(const geometry_msgs::WrenchStamped &wrench_stamped) {
		g_ft_value = wrench_stamped.wrench;
	}

	sensor_msgs::JointState Irb120AccomodationControl::getJointState() {
		
		return g_joint_state;
	}

	geometry_msgs::Wrench Irb120AccomodationControl::getFTSensorValue() {
		return g_ft_value;
	}


