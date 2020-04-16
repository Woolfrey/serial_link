#include "Eigen/Dense"				// Eigen::MatrixXd and inverse() function
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"
#include "serial_link/Serial.h"


geometry_msgs::Point rotatePoint(const geometry_msgs::Quaternion &rot, const geometry_msgs::Point &point)
{
	geometry_msgs::Point p;									// To be returned

	p.x = (rot.w*rot.w + rot.x*rot.x - rot.y*rot.y - rot.z*rot.z)*point.x
            + (2*rot.x*rot.y - 2*rot.w*rot.z)*point.y
 	    + (2*rot.x*rot.z + 2*rot.w*rot.y)*point.z;

	p.y = (2*rot.x*rot.y +rot.w*rot.z)*point.x
	    + (rot.w*rot.w - rot.x*rot.x + rot.y*rot.y - rot.z*rot.z)*point.y
	    + (2*rot.y*rot.z - 2*rot.w*rot.x)*point.z;

	p.z = (2*rot.x*rot.z - 2*rot.w*rot.y)*point.x
	    + (2*rot.y*rot.z + 2*rot.w*rot.x)*point.y
	    + (rot.w*rot.w - rot.x*rot.x - rot.y*rot.y + rot.z*rot.z)*point.z;

	return p;
}

geometry_msgs::Pose multiplyPose(const geometry_msgs::Pose &T1, const geometry_msgs::Pose &T2)
{
	geometry_msgs::Pose T3;									// To be returned
	geometry_msgs::Point p3;								// Portion of translation to be rotated

	p3 = rotatePoint(T1.orientation, T2.position);						// Rotated portion

	// Add the rotated component to the translation of the first
	T3.position.x = T1.position.x + p3.x;						
	T3.position.y = T1.position.y + p3.y;
	T3.position.z = T1.position.z + p3.z;

	// Quaternion multiplication
	T3.orientation.w = T1.orientation.w*T2.orientation.w - T1.orientation.x*T2.orientation.x - T1.orientation.y*T2.orientation.y - T1.orientation.z*T1.orientation.z;
	T3.orientation.x = T1.orientation.w*T2.orientation.x + T1.orientation.x*T2.orientation.w + T1.orientation.y*T2.orientation.z - T1.orientation.z*T2.orientation.y;
	T3.orientation.y = T1.orientation.w*T2.orientation.y - T1.orientation.x*T2.orientation.z + T1.orientation.y*T2.orientation.w + T1.orientation.z*T2.orientation.x;
	T3.orientation.z = T1.orientation.w*T2.orientation.z + T1.orientation.x*T2.orientation.y - T1.orientation.y*T2.orientation.x + T1.orientation.z*T2.orientation.w;

	return T3;
}

void updateForwardKinematics(geometry_msgs::PoseArray &T, const sensor_msgs::JointState &jointState,
			     const serial_link::Serial &serial, const geometry_msgs::Pose &T0)
{
	const int n = serial.joint.size();							// No. of joints
	geometry_msgs::Pose Tq;									// Transform of the current joint position
	double phi;

	for(int i = 0; i < n; i++)
	{
		// Construct the individual joint transform
		if(serial.joint[i].is_revolute)							
		{
			// Zero translation
			Tq.position.x = 0;
			Tq.position.y = 0;
			Tq.position.z = 0;
			
			// Rotate around the axis
			phi = 0.5*jointState.position[i];
			Tq.orientation.w = cos(phi);
			Tq.orientation.x = sin(phi)*serial.joint[i].axis.x;
			Tq.orientation.y = sin(phi)*serial.joint[i].axis.y;
			Tq.orientation.z = sin(phi)*serial.joint[i].axis.z;
		}
		else
		{
			// Translate along the axis
			Tq.position.x = jointState.position[i]*serial.joint[i].axis.x;
			Tq.position.y = jointState.position[i]*serial.joint[i].axis.y;
			Tq.position.z = jointState.position[i]*serial.joint[i].axis.z;

			// Zero rotation
			Tq.orientation.w = 1;
			Tq.orientation.x = 0;
			Tq.orientation.y = 0;
			Tq.orientation.z = 0;
		}

		if(i == 0) T.poses[i] = multiplyPose(multiplyPose(T0,serial.link[i].transform),Tq); // First multiply the base transform
		else	   T.poses[i] = multiplyPose(multiplyPose(T.poses[i-1],serial.link[i].transform),Tq);
	}

	T.poses[n] = multiplyPose(T.poses[n-1],serial.link[n].transform);		// Transform to end-effector has no joint
}

void updateAxis(Eigen::MatrixXd &axis, const geometry_msgs::PoseArray &FK, const serial_link::Serial &serial)
{
	geometry_msgs::Point point;

	for(int i = 0; i < serial.joint.size(); i++)
	{
		point.x = serial.joint[i].axis.x;
		point.y = serial.joint[i].axis.y;
		point.z = serial.joint[i].axis.z;

		point = rotatePoint(FK.poses[i].orientation, point);
		
		axis(0,i) = point.x;
		axis(1,i) = point.y;
		axis(2,i) = point.z;
	}
}

void updateTranslation(Eigen::MatrixXd &trans, const geometry_msgs::PoseArray &FK)
{
	const int n = FK.poses.size();

	for(int i = 0; i < n; i++)
	{
		trans(0,i) = FK.poses[n].position.x - FK.poses[i].position.x;
		trans(1,i) = FK.poses[n].position.y - FK.poses[i].position.y;
		trans(2,i) = FK.poses[n].position.z - FK.poses[i].position.z;
	}	
}

void updateJacobian(Eigen::MatrixXd &J, const Eigen::MatrixXd &a, const Eigen::MatrixXd &r, const serial_link::Serial serial)
{
	const int n = J.cols();										// No. of joints
	Eigen::Vector3d ai;
	Eigen::Vector3d ri;

	for(int i = 0; i < n; i++)
	{
		ai = a.block(0,i,3,1);
		if(serial.joint[i].is_revolute)
		{
			ri = a.block(0,i,3,1);

			J.block(0,i,3,1) = ai.cross(ri);
			J.block(3,i,3,1) = ai;
		}
		else
		{
			J.block(0,i,3,1) = ai;
			J(3,i) = 0;
			J(4,i) = 0;
			J(5,i) = 0;
		}
	}
}

geometry_msgs::Pose getPoseError(const geometry_msgs::Pose &d, const geometry_msgs::Pose &a)
{
	geometry_msgs::Pose error;
	
	// Position Error
	error.position.x = d.position.x - a.position.x;
	error.position.y = d.position.y - a.position.y;
	error.position.z = d.position.z - a.position.z;

	// Orientation Error
error.orientation.w = d.orientation.w*a.orientation.w + d.orientation.x*a.orientation.x + d.orientation.y*a.orientation.y + d.orientation.z*a.orientation.z;
error.orientation.x = -d.orientation.w*a.orientation.x + d.orientation.x*a.orientation.w - d.orientation.y*a.orientation.z + d.orientation.z*a.orientation.y;
error.orientation.y = -d.orientation.w*a.orientation.y + d.orientation.x*a.orientation.z + d.orientation.y*a.orientation.w - d.orientation.z*a.orientation.x;
error.orientation.z = -d.orientation.w*a.orientation.z - d.orientation.x*a.orientation.y + d.orientation.y*a.orientation.x + d.orientation.z*a.orientation.w;

	return error;
}


double getJointWeight(const double &pos, const double &vel, const serial_link::Joint joint)
{
	double s  = 1000*pow(joint.upper_limit - joint.lower_limit, -2);			// Individual joint scalar
	double u  = joint.upper_limit - pos;							// Distance from upper limit
	double v  = pos - joint.lower_limit;							// Distance from lower limit
	double df = (u*u - v*v)/(-s*u*u*v*v);							// Partial-derivative of penalty function
	double fdot = df*vel;									// Time-derivative of penalty function
	
	if(fdot > 0) return df;									// Moving toward a joint limit
	else return 0;										// Moving away from joint limit
}
						
