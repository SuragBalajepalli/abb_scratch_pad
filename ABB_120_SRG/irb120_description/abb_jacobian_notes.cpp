#include <accomodation_controller.h>

void ftSensorCallBack(geometry_msgs::WrenchStamped ft_value) {
	/* message description:
	std_msgs/Header header
  		uint32 seq
  		time stamp
  		string frame_id
	geometry_msgs/Wrench wrench
  		geometry_msgs/Vector3 force
    		float64 x
    		float64 y
    		float64 z
  		geometry_msgs/Vector3 torque
    		float64 x
    		float64 y
    		float64 z
*/

	
}

void jointPosCallBack(sensor_msgs::JointState joint_state) {
	/* message description:
	 std_msgs/Header header
  		uint32 seq
  		time stamp
  		string frame_id
	string[] name
	float64[] position
	float64[] velocity
	float64[] effort
*/


}

void WrenchToCartVel(geometry_msgs/Wrench wrench, geometry_msgs/Twist &twist) {
 /* twist: 
 		geometry_msgs/Vector3 linear
  			float64 x
  			float64 y
  			float64 z
		geometry_msgs/Vector3 angular
		  	float64 x
		  	float64 y
		  	float64 z

*/
1. wrench to eigen 6x1
2. multiply with Ka (6x6), private member variable
3. convert resulting matrix to twist

}

void CartVelToJointVel (geometry_msgs/Twist twist, Eigen::Matrix6f jacobian, vector<float> &joint_vel) {
	
}