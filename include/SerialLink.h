#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/JointState.h"
#include "Eigen/Dense"
#include "vector"
#include "urdf/model.h"

class SerialLink{

	public:

		int Hz;									// Control frequency (used for joint limit avoidance)



		// Constructor(s)
		SerialLink();								// Default control frequency of 100Hz
		SerialLink(int controlFreq);						// Specify the control frequency

		// Get Functions
		double getManipulability();						// Get the measure of manipulability at this pose

		geometry_msgs::Transform getEndEffectorTF();				// Get the end-effector transform for the current joint state, origin frame
		

		// Set Functions
		void setDampingThreshold(double &input);				// Set the damping threshold for activating Damped Least Squares
		void setDGain(double &input);						// Set the D gain for PD
		void setMaxDamping(double &input);					// Set the maximum damping fact for Damped Least Squares
		void setOrigin(geometry_msgs::Transform &input);			// Set the origin frame for computing kinematics and dynamics
		void setPGain(double &input);						// Set the P gain for P, PD control

		void updateState(sensor_msgs::JointState &input);			// Update just the joint states
		void updateState(sensor_msgs::JointState &input, geometry_msgs::Transform &origin); // Update joint states and base transform
		
		// Velocity Control Functions

		sensor_msgs::JointState rmrc(geometry_msgs::Pose &pose);		// Move the end-effector to a desired pose
		sensor_msgs::JointState rmrc(geometry_msgs::Twist &velocity,		// Track Cartesian trajectory
					     geometry_msgs::Pose &pose);

		sensor_msgs::JointState rmrc(geometry_msgs::Twist &velocity,
					     geometry_msgs::Pose &pose,
		   			     sensor_msgs::JointState &redundant); 	// Track a trajectory with redundant tas


	private:

		Eigen::MatrixXd a;							// Axis of actuation for each joint
		Eigen::MatrixXd r;							// Distance from each joint to end-effector		

};											// Class definitions must end with a semicolon


SerialLink::SerialLink()
{
	ROS_INFO_STREAM("Default control frequency is 100Hz. Is this OK? (Y/n):");
	char input;
	std::cin >> input;
	if(input == 'Y')
	{
		SerialLink(100);
	}
}

SerialLink::SerialLink(int controlFreq)
{
	this->Hz = controlFreq;

	// Step 1: Prune the URDF tree to get rid of all superfluous links
	urdf::Model model;									// Model from urdf file
	std::vector<urdf::Link> branch;								// Gets the main forward kinematics sequence
	std::vector<urdf::Link> twig;								// All child links attached to a branch
	std::vector<urdf::Link> bud;								// All child links attached to a twig
	urdf::Vector3 tempVector;								// Temporary storage
	urdf::Rotation tempRotation;								// Temporary storage
	geometry_msgs::Transform tempTF;							// Temporary storage

	if(model.initParam("urdf"))
	{
		branch.push_back(*model.getRoot());						// This gets the location of the urdf model
		int count = 0;									// No. of child links	

		bool breakLoop = false;

		while(!breakLoop)
		{
			twig.clear();								// Clear twig array
			if(branch[count].child_links.size() == 0)				// If no more child links...
			{
				breakLoop = true;						// ... break the loop
			}
			else
			{
				twig.resize(branch[count].child_links.size());			// Get number of child links attached to this branch
				std::vector<int> potential(twig.size(),0);			// Potential for each of the twigs

				for(int i=0; i<twig.size(); i++)
				{
					twig[i]  = *branch[count].child_links[i];		// Add child link
					bud.clear();

					for(int j=0; j<twig[i].child_links.size(); j++)
					{
						bud.push_back(*twig[i].child_links[j]); 	// Add child links from this twig
						
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

	// Step 2: Assign values 

}
