#ifndef TELLO_ROS_H
#define TELLO_ROS_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include "tello_ros/tello.h"


namespace tello_ros
{
class TelloROS
{
public:
    TelloROS(const ros::NodeHandle& nh);

private:
    std::shared_ptr<Tello> tello_ptr_;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::CameraPublisher camera_pub_;
    ros::ServiceServer takeoff_srv_;
    ros::ServiceServer land_srv_;
    ros::ServiceServer flip_r_srv_;
    ros::ServiceServer flip_l_srv_;
    ros::ServiceServer flip_b_srv_;
    ros::ServiceServer flip_f_srv_;
    ros::ServiceServer hover_srv_;
    ros::ServiceServer emergency_srv_;
    ros::ServiceServer enable_stream_srv_;
    ros::ServiceServer disable_stream_srv_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher imu_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher temperature_pub_;
    ros::Publisher height_pub_;
    ros::Publisher barometer_pub_;
    ros::Timer timer_;
    ros::Timer camera_timer_;
    sensor_msgs::CameraInfo info_msg_;

    bool takeoffCallback(std_srvs::Empty::Request  &,
                         std_srvs::Empty::Response &);

    bool landCallback(std_srvs::Empty::Request  &,
                      std_srvs::Empty::Response &);

    bool flipRCallback(std_srvs::Empty::Request  &,
                       std_srvs::Empty::Response &);

    bool flipLCallback(std_srvs::Empty::Request  &,
                       std_srvs::Empty::Response &);

    bool flipFCallback(std_srvs::Empty::Request  &,
                       std_srvs::Empty::Response &);

    bool flipBCallback(std_srvs::Empty::Request  &,
                       std_srvs::Empty::Response &);

    bool hoverCallback(std_srvs::Empty::Request  &,
                       std_srvs::Empty::Response &);

    bool emergencyCallback(std_srvs::Empty::Request  &,
                           std_srvs::Empty::Response &);

    bool enableStreamCallback(std_srvs::Empty::Request  &,
                              std_srvs::Empty::Response &);

    bool disableStreamCallback(std_srvs::Empty::Request  &,
                               std_srvs::Empty::Response &);

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent&);

    void cameraLoop(const ros::TimerEvent&);

};
}  // namespace tello_ros
#endif //TELLO_ROS_H
