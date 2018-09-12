#include <irb120_accomodation_control/irb120_accomodation_control.h>
int main(int argc, char** argv) {
	ros::init(argc, argv, "transform_test");
	ros::NodeHandle nh;
	Irb120AccomodationControl control(nh);
	geometry_msgs::TransformStamped tf;
	geometry_msgs::Wrench ft_sensor_value;
	Eigen::Affine3f testing;
	Eigen::Vector3f testing_vector = Eigen::Vector3f::Zero();
	Eigen::Vector3f result_vector;
	while(ros::ok()) {
		cout<<"Zero vector on multiplication with affine"<<endl;
		testing = control.getAffine_test();
		result_vector = testing * testing_vector;
		cout<<testing(0,0)<<"\t"<<testing(0,1)<<"\t"<<testing(0,2)<<"\n";
		cout<<testing(1,0)<<"\t"<<testing(1,1)<<"\t"<<testing(1,2)<<"\n";
		cout<<testing(2,0)<<"\t"<<testing(2,1)<<"\t"<<testing(2,2)<<"\n";
		cout<<"multiplied with"<<endl;
		cout<<testing_vector<<endl;
		cout<<"iS"<<endl;
		cout<<result_vector<<endl;
		//tf = control.getFlangeTransform();
		//ROS_INFO_STREAM(" CURRENT TRANSFORM "<<tf);
		//ft_sensor_value = control.getTransformedWrench();
		//ROS_INFO_STREAM("Recieved FT value at base"<<ft_sensor_value);
		//ft_sensor_value = control.getFTSensorValue();
		//ROS_INFO_STREAM("Recieved FT value at tool"<<ft_sensor_value);
		//control.findCartVelFromWrench(ft_sensor_value, end_effector_twist);
		//ROS_INFO_STREAM("Calculated twist is: "<<end_effector_twist);  
		ros::Duration(1).sleep();
	}
}