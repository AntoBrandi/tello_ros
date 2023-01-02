#include "tello_ros/tello_ros.h"


namespace tello_ros
{
TelloROS::TelloROS(const ros::NodeHandle& nh) : nh_(nh)
{
    tello_ptr_ = std::make_shared<Tello>();
    if (!tello_ptr_->Bind())
    {
        ROS_ERROR("Cannot connect to Tello");
    }


    takeoff_srv_ = nh_.advertiseService("takeoff", &TelloROS::takeoffCallback, this);
    land_srv_ = nh_.advertiseService("land", &TelloROS::landCallback, this);
    flip_r_srv_ = nh_.advertiseService("flip_r", &TelloROS::flipRCallback, this);
    flip_l_srv_ = nh_.advertiseService("flip_l", &TelloROS::flipLCallback, this);
    flip_f_srv_ = nh_.advertiseService("flip_f", &TelloROS::flipFCallback, this);
    flip_b_srv_ = nh_.advertiseService("flip_b", &TelloROS::flipBCallback, this);
    hover_srv_ = nh_.advertiseService("hover", &TelloROS::hoverCallback, this);
    emergency_srv_ = nh_.advertiseService("emergency", &TelloROS::emergencyCallback, this);
    cmd_vel_sub_ = nh_.subscribe("chatter", 1000, &TelloROS::cmdVelCallback, this);
}


bool TelloROS::takeoffCallback(std_srvs::Empty::Request  &,
                               std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("takeoff");
}


bool TelloROS::landCallback(std_srvs::Empty::Request  &,
                            std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("land");
}


bool TelloROS::flipRCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("flip r");
}


bool TelloROS::flipLCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("flip l");
}


bool TelloROS::flipFCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("flip f");
}


bool TelloROS::flipBCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("flip b");
}


bool TelloROS::hoverCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("stop");
}


bool TelloROS::emergencyCallback(std_srvs::Empty::Request  &,
                                 std_srvs::Empty::Response &)
{
    return tello_ptr_->SendCommand("emergency");
}


void TelloROS::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    std::string cmd = "rc ";
    
    int yaw_velocity = std::max(-100, std::min(100, (int)msg->angular.z));
    int forward_backward_velocity = std::max(-100, std::min(100, (int)msg->linear.x));
    int left_right_velocity = std::max(-100, std::min(100, (int)msg->linear.y));
    int up_down_velocity = std::max(-100, std::min(100, (int)msg->linear.z));

    cmd += std::to_string(left_right_velocity) + " " +
           std::to_string(forward_backward_velocity) + " " +
           std::to_string(up_down_velocity) + " " +
           std::to_string(yaw_velocity);

    tello_ptr_->SendCommand(cmd);
}
} // namespace tello_ros
