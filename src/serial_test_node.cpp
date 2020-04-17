#include "ros/ros.h"
#include "SerialLink.h"
#include "string"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_link_example");			// Create node
	ros::NodeHandle n;

	SerialLink robot("ur3");					// Create serial link object


	// Arbitrary joint state information to test
	sensor_msgs::JointState joint_state;
	joint_state.position.resize(robot.n);
	joint_state.velocity.resize(robot.n);
	joint_state.effort.resize(robot.n);
	for(int i = 0; i < robot.n; i++)
	{
		joint_state.position[i] = 0.0;
		joint_state.velocity[i] = 0.0;
		joint_state.effort[i] = 0.0;
	}

	robot.updateState(joint_state);					// Update the robot state

	// Check the forward kinematics calcs
	std::cout << " " << std::endl;
	std::cout << "Forward Kinematics:" << std::endl;
	std::cout << " " << std::endl;
	for(int i = 0; i < robot.n; i++)
	{
		std::cout << "x: " << robot.FK.poses[i].position.x << std::endl;
		std::cout << "y: " << robot.FK.poses[i].position.y << std::endl;
		std::cout << "z: " << robot.FK.poses[i].position.z << std::endl;
		std::cout << " " << std::endl;
	}

	// Check that resolved motion rate control is working
	geometry_msgs::Pose pose;					// Arbitrary pose
	pose.position.x = 0.5;
	pose.position.y = 0.1;
	pose.position.z = 0.2;
	pose.orientation.w = 1.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;

	geometry_msgs::Twist twist;					// Arbitrary velocity
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
	twist.linear.z = 0.0;
	twist.angular.x = 0.0;
	twist.angular.y = 0.0;
	twist.angular.z = 0.0;

	sensor_msgs::JointState control = robot.rmrc(pose, twist);
	std::cout << "Joint velocities for Cartesian control:" << std::endl;
	for(int i = 0; i < robot.n; i++)
	{
		std::cout << control.velocity[i] << " ";
	}
	std::cout << std::endl;
	std::cout << std::endl;
	return 0;					// No problems with main()
}
