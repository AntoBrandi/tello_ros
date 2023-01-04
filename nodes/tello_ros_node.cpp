#include "tello_ros/tello_ros.h"
#include <ros/ros.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tello_ros_node");
    ros::NodeHandle nh("tello");
    tello_ros::TelloROS tello(nh);

    ros::spin();

    return 0;
}