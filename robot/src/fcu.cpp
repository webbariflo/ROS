#include "functions.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "fcu_node");
    ros::NodeHandle fcu_node;

    init_sub_pub(fcu_node);

    wait4connect();

    wait4start();

}