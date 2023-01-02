#include "tello_ros/tello.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tello_ros_node");
    tello_ros::Tello tello;
    if (!tello.Bind())
    {
        return 0;
    }

    tello.SendCommand("takeoff");
    tello.SendCommand("land");
    ros::spin();

    return 0;
}