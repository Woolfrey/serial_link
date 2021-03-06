#include "geometry_msgs/Twist.h"
#include "Kinematics.h"
#include "string"
#include "urdf/model.h"

class SerialLink{

	public:
		// Properties
		double p_gain = 1;
		double d_gain = 0.1;
		double threshold = 0.01;					// Threshold value for activating damped least squares
		double max_damping = 0.1;					// Maximum damping factor

		int control_frequency = 100;					// Control frequency (used for joint limit avoidance)
		int n;								// No. of joints

		Eigen::MatrixXd J;						// Jacobian matrix
		Eigen::MatrixXd invJ;						// Inverse of the Jacobian

		geometry_msgs::PoseArray FK;					// Forward kinematics chain

		sensor_msgs::JointState joint_state;				// Position, velocity, acceleration?

		serial_link::Serial serial;					// Array of links and joints

		std::string name = "manipulator_name";				// Used to identify this manipulator

		// Constructor(s)
		SerialLink();							// Empty constructor
		SerialLink(const std::string &paramName);			// Contructor from parameter server
		

		// Get Functions
		double getDamping(const Eigen::MatrixXd &J);			// Scalar value for damped least squares

		sensor_msgs::JointState rmrc(const geometry_msgs::Pose &pose, 
					     const geometry_msgs::Twist &velocity);

		void getJacobian(Eigen::MatrixXd &J);

		// Set Functions
		bool init(const std::string &paramName);			// Initialize from parameter server		
		void setOrigin(const geometry_msgs::Pose &input);	// Set the origin frame for computing kinematics and dynamics
		void updateState(const sensor_msgs::JointState &input);		// Update just the joint states
		void updateState(const sensor_msgs::JointState &input,
				 const geometry_msgs::Pose &baseTF); 		// Update joint states and base transform

	private:
		Eigen::MatrixXd a;						// Axis of actuation for each joint
		Eigen::MatrixXd r;						// Distance from each joint to end-effector

		geometry_msgs::Pose origin;					// Origin with which to compute kinematics, dynamics

		sensor_msgs::JointState control_msg;				// Joint control message

		void invertJacobian(Eigen::MatrixXd &W);			// Weighted pseudo-inverse
		void invertJacobian();						// No weighting matrix given, so use identity matrix


};										// Class definitions must end with a semicolon

SerialLink::SerialLink() 							// Empty constructor
{
}

SerialLink::SerialLink(const std::string &paramName)				// Name in the parameter server
{
	init(paramName);							
}

bool SerialLink::init(const std::string &paramName)
{
	// Step 1: Prune the URDF tree to get rid of all superfluous links
	urdf::Model model;								// Model from urdf file
	std::vector<urdf::Link> branch;							// Gets the main forward kinematics sequence
	std::vector<urdf::Link> twig;							// All child links attached to a branch
	std::vector<urdf::Link> bud;							// All child links attached to a twig
	urdf::Vector3 tempVector;							// Temporary storage
	urdf::Rotation tempRotation;							// Temporary storage
	geometry_msgs::Pose tempTF;							// Temporary storage

	if(model.initParam(paramName)) 
	{
		branch.push_back(*model.getRoot());					// This gets the location of the urdf model
		int count = 0;								// No. of child links	

		bool breakLoop = false;

		while(!breakLoop)
		{
			twig.clear();							// Clear twig array
			if(branch[count].child_links.size() == 0)			// If no more child links...
			{
				breakLoop = true;					// ... break the loop
			}
			else
			{
				twig.resize(branch[count].child_links.size());		// Get number of child links attached to this branch
				std::vector<int> potential(twig.size(),0);		// Potential for each of the twigs

				for(int i=0; i<twig.size(); i++)
				{
					twig[i]  = *branch[count].child_links[i];	// Add child link
					bud.clear();

					for(int j=0; j<twig[i].child_links.size(); j++)
					{
						bud.push_back(*twig[i].child_links[j]); // Add child links from this twig
						
						for(int k=0; k < bud.size(); k++) potential[i] += bud[k].child_links.size(); // Twig potential is number of child links
					}
				}
				int index;
				int max = 0;
				for(int i=0; i<twig.size(); i++)
				{
					if(potential[i] > max)
					{
						index = i;
						max = potential[i];
					}
				}
				if(max == 0)							// Tree doesn't go any further
				{
					for(int i=0; i<twig.size(); i++)
					{
						if(twig[i].child_links.size() > 0)		// Find the twig with 1 child
						{
							branch.push_back(twig[i]);		// Add this twig
							branch.push_back(*twig[i].child_links[0]); // This should be the end-effector
						}
					}
					breakLoop = true;
				}
				else
				{
					branch.push_back(twig[index]);
					count++;
				}
			}
		}
	}
	else throw "Unable to load URDF from the parameter server";

	ROS_INFO_STREAM("The name of this robot is " << model.getName() << ".");
	this->name = model.getName();					// Transfer name


	// Cut out superfluous "base" links etc.
	std::vector<urdf::Link> temp;
	for(int i = branch.size()-1; i > 0; i--)			// Start from the end-effector and work back
	{
		int type = branch[i].parent_joint->type;		// 1 revolute, 2 continuous, 3 prismatic
		if(type == 1 || type == 2 || type == 3)	temp.push_back(branch[i]); // Add to the array
	}
	std::reverse(temp.begin(), temp.end());				// Reverse the array to get the forward sequence
	this->n = temp.size();						// No. of actual joints

	ROS_INFO_STREAM("There were " << this->n << " joints detected in this model.");

	// Step 2: Transfer information from urdf to the custom Link class
	this->serial.joint.resize(this->n);
	this->serial.link.resize(this->n);

	for(int i = 0; i < this->n; i++)
	{
		// Transfer joint information
		if(temp[i].parent_joint->type == 1 || 2)					// Revolute joint
		{
			this->serial.joint[i].is_revolute = 1;
		}
		else if(temp[i].parent_joint->type == 3)					// Prismatic joint
		{
			this->serial.joint[i].is_revolute = 0;
		}
		this->serial.joint[i].damping	   = temp[i].parent_joint->dynamics->damping;
		this->serial.joint[i].friction     = temp[i].parent_joint->dynamics->friction;
		this->serial.joint[i].torque_limit = temp[i].parent_joint->limits->effort;
		this->serial.joint[i].speed_limit  = temp[i].parent_joint->limits->velocity;
		this->serial.joint[i].upper_limit  = temp[i].parent_joint->limits->upper;
		this->serial.joint[i].lower_limit  = temp[i].parent_joint->limits->lower;

		// Transfer dynamic properties
		this->serial.link[i].inertia.m     = temp[i].inertial->mass;
		this->serial.link[i].inertia.ixx   = temp[i].inertial->ixx;
		this->serial.link[i].inertia.ixy   = temp[i].inertial->ixy;
		this->serial.link[i].inertia.ixz   = temp[i].inertial->ixz;
		this->serial.link[i].inertia.iyy   = temp[i].inertial->iyy;
		this->serial.link[i].inertia.iyz   = temp[i].inertial->iyz;
		this->serial.link[i].inertia.izz   = temp[i].inertial->izz;
		this->serial.link[i].inertia.com.x = temp[i].inertial->origin.position.x;
		this->serial.link[i].inertia.com.y = temp[i].inertial->origin.position.y;
		this->serial.link[i].inertia.com.z = temp[i].inertial->origin.position.z;

		// Transfer geometric information
		this->serial.link[i].transform.position.x    = temp[i].parent_joint->parent_to_joint_origin_transform.position.x;
		this->serial.link[i].transform.position.y    = temp[i].parent_joint->parent_to_joint_origin_transform.position.y;
		this->serial.link[i].transform.position.z    = temp[i].parent_joint->parent_to_joint_origin_transform.position.z;
		this->serial.link[i].transform.orientation.w = temp[i].parent_joint->parent_to_joint_origin_transform.rotation.w;
		this->serial.link[i].transform.orientation.x = temp[i].parent_joint->parent_to_joint_origin_transform.rotation.x;
		this->serial.link[i].transform.orientation.y = temp[i].parent_joint->parent_to_joint_origin_transform.rotation.y;
		this->serial.link[i].transform.orientation.z = temp[i].parent_joint->parent_to_joint_origin_transform.rotation.z;
		this->serial.joint[i].axis.x = temp[i].parent_joint->axis.x;
		this->serial.joint[i].axis.y = temp[i].parent_joint->axis.y;
		this->serial.joint[i].axis.z = temp[i].parent_joint->axis.z;
	}

	ROS_INFO("Finished extracting serial-link properties from urdf.");

	// Step 3: Resize arrays based on no. of joints
	this->FK.poses.resize(this->n);				// 1 transform for each joint frame + 1 to the end-effector???

			
	this->a.resize(3,this->n);				// Axis of rotation for each joint in global frame
	this->r.resize(3,this->n);				// Translation from joint to end-effector
	this->J.resize(6,this->n);				// Jacobian matrix


	this->control_msg.velocity.resize(this->n);
	this->control_msg.effort.resize(this->n);
	this->joint_state.position.resize(this->n);
	this->joint_state.velocity.resize(this->n);
	this->joint_state.effort.resize(this->n);
	

	// Step 4: Assign default values
	this->origin.position.x = 0.0;
	this->origin.position.y = 0.0;
	this->origin.position.z = 0.0;
	this->origin.orientation.w = 1.0;
	this->origin.orientation.x = 0.0;
	this->origin.orientation.y = 0.0;
	this->origin.orientation.z = 0.0;

	for(int i = 0; i < this->n; i++)
	{
		this->joint_state.position[i] = 0.0;
		this->joint_state.velocity[i] = 0.0;
		this->joint_state.effort[i] = 0.0;
	}

	// Step 5: Get custom values from parameter server
	if(ros::param::get(this->name+"/control_frequency", this->control_frequency));
	else ROS_INFO_STREAM("Using default control_frequency of " << this->control_frequency << " Hz.");

	if(ros::param::get(this->name+"/p_gain", this->p_gain));
	else ROS_INFO_STREAM("Using default p_gain of " << this->p_gain << ".");

	if(ros::param::get(this->name+"/d_gain", this->d_gain));
	else ROS_INFO_STREAM("Using default d_gain of " << this->d_gain << ".");

	if(ros::param::get(this->name+"/max_damping", this->max_damping));
	else ROS_INFO_STREAM("Using default max_damping of " << this->max_damping << " for singularity avoidance.");

	if(ros::param::get(this->name+"/threshold", this->threshold));
	else ROS_INFO_STREAM("Using default threshold of " << this->threshold << " for activating singularity avoidance.");

/*	This is old code that can be deleted in the future
	// GET JOINTS, JOINT TRANSFORMS, AND INERTIA ARRAY
	this->jointArray.clear();
	this->inertiaArray.clear();
	this->jointTF.clear();

	for(int i=branch.size()-1; i>0; i--)
	{
		int type = branch[i].parent_joint->type;
		if(type == 1 || type == 2 || type == 3) 
		{
			this->jointArray.push_back(*branch[i].parent_joint);			// Add to joint array
			this->inertiaArray.push_back(*branch[i].inertial);			// Add to inertia array
		}

		if(i > 1)									// Add all but first joint tf	
		{
			tempVector = branch[i].parent_joint->parent_to_joint_origin_transform.position;
			tempRotation = branch[i].parent_joint->parent_to_joint_origin_transform.rotation;
			tempTF.setOrigin(tf::Vector3(tempVector.x, tempVector.y, tempVector.z));
			tempTF.setRotation(tf::Quaternion (tempRotation.x, tempRotation.y, tempRotation.z, tempRotation.w));
			this->jointTF.push_back(tempTF);
		}
		
	}

	ROS_INFO_STREAM("There were " << jointArray.size() << " joints detected in this robot model.");
	
	std::reverse(jointArray.begin(),jointArray.end());				// Reverse joint vector
	std::reverse(jointTF.begin(), jointTF.end());					// Reverse jointTF vector
	std::reverse(inertiaArray.begin(), inertiaArray.end());				// Reverse inertia array

	this->dof = jointArray.size();							// No. of joints

	// SET SIZE OF MATRICES AND ARRAYS BASED ON MODEL PROPERTIES

	// Eigen::MatrixXd -> Kinematics
	this->a.setZero(3,this->dof);							// Axis of actuation for each joint
	this->adot.setZero(3,this->dof);						// Time derivative for axis
	this->c.setZero(3,this->dof);							// Location for centre of mass
	this->r.setZero(3,this->dof);							// Distance from joint to end-effector
	this->ldot.setZero(3,this->dof);
	this->w.setZero(3,this->dof);
	this->J.setZero(6,this->dof);							// Jacobian matrix
	this->JJt.setZero(6,6);								// J*J'
	this->invJ.setZero(this->dof,6);						// inv(W)*J'*inv(J*inv(W)*J')
	this->Jdot.setZero(6,this->dof);						// Time-derivative of Jacobian matrix
	this->N.setZero(this->dof,this->dof);						// Null space projection matrix = I - invJ*J
	this->W.setIdentity(this->dof,this->dof);					// Joint weighting matrix
	this->jointWeight.resize(this->dof);						// Elements in the joint weight matrix
	for(int i=0; i<dof; i++) jointWeight[i] = 0.0;					// Set initial values to zero

	this->I6.setIdentity();								// 6x6 identity matrix
	this->I.setIdentity(this->dof,this->dof);					// nxn Identity matrix

	// Eigen::MatrixXd -> Dynamics
	this->Jm.setZero(6*this->dof,this->dof);					// Jacobian to c.o.m. for all joints
	this->Jmt.setZero(this->dof,6*this->dof);					
	this->JmtH.setZero(this->dof,6*this->dof);
	this->Jmdot.setZero(6*this->dof,this->dof);					
	this->H.setZero(6*this->dof,6*this->dof);					// Cartesian mass-inertia matrix for all joints
	this->Hdot.setZero(6*this->dof,6*this->dof);					// Time derivative of Cartesian mass-inertia matrix
	this->M.setZero(this->dof,this->dof);						// M = Jm'*H*Jm is the inertia matrix in joint space
	this->C.setZero(this->dof,this->dof);						// C = Jm'*(H*Jmdot + Hdot*Jm) 
	this->g.setZero(this->dof);							// Gravitational torque g = Jm'*wg

	// sensor_msgs::JointState
	this->jointState.position.resize(this->dof);						
	this->jointState.velocity.resize(this->dof);
	this->jointState.effort.resize(this->dof);

	// std_msgs::Float32MultiArray
	this->xdot.data.resize(6);							// End-effector velocity command
	this->xddot.data.resize(6);							// End-effector acceleration
	this->qddot.data.resize(this->dof);						// Joint accelerations
	this->qdot_r.data.resize(this->dof);						// Redundant task, velocity level
	this->qddot_r.data.resize(this->dof);						// Redundant task, acceleration level

	//tf::Matrix3x3
	this->Kp.setIdentity();								// Proportional gains set initially to 1
	this->Ki.setValue(0,0,0,0,0,0,0,0,0);						// Integral gains set initially to 0
	this->Kd.setValue(3,0,0,0,3,0,0,0,3);						// Kd^2 >= 4Kp for stability

	//tf::Transform
	this->originFrame.setIdentity();						// Base transform initially set to zero
	this->FK.resize(this->dof+1);							// This gives 1 tf for each joint, plus the end-effector

*/
}

void SerialLink::setOrigin(const geometry_msgs::Pose &input)	// Set the origin frame for computing kinematics and dynamics
{
	this->origin = input;							
}

void SerialLink::updateState(const sensor_msgs::JointState &input,
			     const geometry_msgs::Pose &baseTF) 			// Update joint states and base transform
{
	setOrigin(baseTF);								// Update base
	updateState(input);
}

void SerialLink::updateState(const sensor_msgs::JointState &input)			// Update just the joint states
{
	this->joint_state.position = input.position;
	this->joint_state.velocity = input.velocity;
	this->joint_state.effort = input.effort;

	updateForwardKinematics(this->FK, this->joint_state, this->serial, this->origin); // Update the forward-kinematics chain
	updateAxis(this->a, this->FK, this->serial);				// Joint axis in reference frame
	updateTranslation(this->r, this->FK);					// Distance from joint to end-effector in reference frame
}

sensor_msgs::JointState SerialLink::rmrc(const geometry_msgs::Pose &pose, const geometry_msgs::Twist &vel)
{
	updateJacobian(this->J, this->a, this->r, this->serial);			// Update the Jacobian matrix for current state

	invertJacobian(); /*** In future use invertJacobian(this->M) ***/		// Update the inverse of this Jacobian

	geometry_msgs::Pose error = getPoseError(pose, this->FK.poses[this->n-1]);	// Compute the current pose error

	for(int i = 0; i < this->n; i++)
	{
		this->control_msg.velocity[i] = this->invJ(i,0)*(vel.linear.x + this->p_gain*error.position.x)
				    	      + this->invJ(i,1)*(vel.linear.y + this->p_gain*error.position.y)
				    	      + this->invJ(i,2)*(vel.linear.z + this->p_gain*error.position.z)
				    	      + this->invJ(i,3)*(vel.angular.x + this->p_gain*error.orientation.x)
				    	      + this->invJ(i,4)*(vel.angular.y + this->p_gain*error.orientation.y)
				    	      + this->invJ(i,5)*(vel.angular.z + this->p_gain*error.orientation.z);
	}

	return control_msg;
}

double SerialLink::getDamping(const Eigen::MatrixXd &J)
{
	Eigen::Matrix<double,6,6> JJt;

	for(int i = 0; i < 6; i++)
	{
		for(int j = 0; j < this->n; j++)
		{
			for(int k = 0; k < 6; k++) 
			{
				JJt(i,k) = J(i,j)*J(k,j); 				// Is this faster than calling J*J.transpose()?
			}
		}
	}

	double m = pow(JJt.determinant(),0.5);						// Measure of manipulability
	
	if(m < this->threshold) return (1-pow(m/this->threshold,2))*this->max_damping;
	else			return 0.0;
}

void SerialLink::invertJacobian(Eigen::MatrixXd &W)					// Weighted pseudo-inverse
{
	double lambda = getDamping(this->J);						// Get the coefficient for DLS

	if(this->n < 7)
	{
		this->invJ = (J*J.transpose() + lambda*Eigen::MatrixXd::Identity(6,6)).inverse(); // Just use damped least squares
	}
	else
	{
		// Update weighting matrix for joint limit avoidance
		for(int i = 0; i < this->n; i++)
		{
			W(i,i) += getJointWeight(this->joint_state.position[i],
						this->joint_state.velocity[i],
						this->serial.joint[i]);
		}

		Eigen::MatrixXd invWJt = W.inverse()*J.transpose();	// Hopefully makes computations a little faster
		this->invJ = invWJt*(J*invWJt + lambda*Eigen::MatrixXd::Identity(6,6)).inverse(); // Weighted pseudoinverse
	}
}

void SerialLink::invertJacobian()							// No weighting matrix given, so use identity matrix
{
	Eigen::MatrixXd W;
	W.setIdentity(this->n,this->n);
	invertJacobian(W);
}


