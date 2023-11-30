
#include <functions3.hpp>
//include API 

int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "rover_node");
	ros::NodeHandle rover_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(rover_node);

  	disarm();
}