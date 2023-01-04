#include "tello_ros/tello_ros.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>


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
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &TelloROS::cmdVelCallback, this);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1000);
    battery_pub_ = nh_.advertise<std_msgs::Float32>("battery", 1000);
    temperature_pub_ = nh_.advertise<std_msgs::Float32>("temperature", 1000);
    height_pub_ = nh_.advertise<std_msgs::Float32>("height", 1000);
    barometer_pub_ = nh_.advertise<std_msgs::Float32>("barometer", 1000);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TelloROS::timerCallback, this);
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


void TelloROS::timerCallback(const ros::TimerEvent&)
{
    // Create and Publish the IMU message
    std::map<std::string,std::string> state = tello_ptr_->GetState();
    tf2::Quaternion orientation;
    orientation.setRPY(std::stod(state["roll"]), 
                       std::stod(state["pitch"]),
                       std::stod(state["yaw"]));
    sensor_msgs::Imu imu;
    imu.header.frame_id = "imu";
    imu.linear_acceleration.x = std::stod(state["agx"])/100;
    imu.linear_acceleration.y = std::stod(state["agy"])/100;
    imu.linear_acceleration.z = std::stod(state["agz"])/100;
    imu.angular_velocity.x = std::stod(state["vgx"]);
    imu.angular_velocity.y = std::stod(state["vgy"]);
    imu.angular_velocity.z = std::stod(state["vgz"]);
    imu.orientation.x = orientation.getX();
    imu.orientation.y = orientation.getY();
    imu.orientation.z = orientation.getZ();
    imu.orientation.w = orientation.getW();
    imu.header.stamp = ros::Time::now();
    imu_pub_.publish(imu);

    // Create and publish Battery
    std_msgs::Float32 battery;
    battery.data = std::stod(state["bat"]);
    battery_pub_.publish(battery);

    // Create and publish Temperature
    std_msgs::Float32 temperature;
    temperature.data = (std::stod(state["templ"]) + std::stod(state["temph"])) / 2;
    temperature_pub_.publish(temperature);

    // Create and publish Height
    std_msgs::Float32 height;
    height.data = std::stod(state["h"]) / 100;
    height_pub_.publish(height);

    // Create and publish Barometer
    std_msgs::Float32 barometer;
    barometer.data = std::stod(state["baro"]) / 100;
    barometer_pub_.publish(barometer);
    
}
} // namespace tello_ros
