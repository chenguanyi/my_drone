#include "bluetooth/bluetooth_publisher.h"
#include <ros/ros.h>

int main(int argc,char** argv)
{
    ros::init(argc,argv,"bluetooth_uart");
    ros::NodeHandle nh("~");

    BlueTooth_Publisher::cma_inparam came = {1101,1100,299.162,207.625};
    BlueTooth_Publisher node(nh,came);
    node.BlueTooth_init(100,"map","laser_link");
    node.run();

}
