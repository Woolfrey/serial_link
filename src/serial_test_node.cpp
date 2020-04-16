#include "ros/ros.h"
#include "SerialLink.h"
#include "string"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_link");
	ros::NodeHandle n;

	try
	{
		SerialLink ur3("ur3");
	}
	catch(const char response)
	{
		std::cout << response << std::endl;
	}

	while(ros::ok())
	{

	}
	
	return 0;					// No problems with main()
}
