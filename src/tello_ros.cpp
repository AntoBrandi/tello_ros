#include "tello_ros/tello_ros.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>


namespace tello_ros
{
TelloROS::TelloROS(const ros::NodeHandle& nh) 
    : nh_(nh)
    , it_(nh)
{
    tello_ptr_ = std::make_shared<Tello>();
    if (!tello_ptr_->Bind())
    {
        ROS_ERROR("Cannot connect to Tello");
    }

    // ROS Interface
    takeoff_srv_ = nh_.advertiseService("takeoff", &TelloROS::takeoffCallback, this);
    land_srv_ = nh_.advertiseService("land", &TelloROS::landCallback, this);
    flip_r_srv_ = nh_.advertiseService("flip_r", &TelloROS::flipRCallback, this);
    flip_l_srv_ = nh_.advertiseService("flip_l", &TelloROS::flipLCallback, this);
    flip_f_srv_ = nh_.advertiseService("flip_f", &TelloROS::flipFCallback, this);
    flip_b_srv_ = nh_.advertiseService("flip_b", &TelloROS::flipBCallback, this);
    hover_srv_ = nh_.advertiseService("hover", &TelloROS::hoverCallback, this);
    emergency_srv_ = nh_.advertiseService("emergency", &TelloROS::emergencyCallback, this);
    enable_stream_srv_ = nh_.advertiseService("enable_stream", &TelloROS::enableStreamCallback, this);
    disable_stream_srv_ = nh_.advertiseService("disable_stream", &TelloROS::disableStreamCallback, this);
    cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1000, &TelloROS::cmdVelCallback, this);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1000);
    battery_pub_ = nh_.advertise<std_msgs::Float32>("battery", 1000);
    temperature_pub_ = nh_.advertise<std_msgs::Float32>("temperature", 1000);
    height_pub_ = nh_.advertise<std_msgs::Float32>("height", 1000);
    barometer_pub_ = nh_.advertise<std_msgs::Float32>("barometer", 1000);
    camera_pub_ = it_.advertiseCamera("camera", 2);
    timer_ = nh_.createTimer(ros::Duration(0.1), &TelloROS::timerCallback, this);
    camera_timer_ = nh_.createTimer(ros::Duration(0.001), &TelloROS::cameraLoop, this, false, false);

    // Init the camera info message
    info_msg_.distortion_model = "plumb_bob";
    info_msg_.height = TELLO_CAMERA_HEIGHT;
    info_msg_.width = TELLO_CAMERA_WIDTH;
    float focal = 0.5 * TELLO_CAMERA_WIDTH / tan(0.5 * TELLO_CAMERA_FOV);
    info_msg_.K[0] = focal;
    info_msg_.K[4] = focal;
    info_msg_.K[2] = info_msg_.width * 0.5;
    info_msg_.K[5] = info_msg_.height * 0.5;
    info_msg_.K[8] = 1.;
    info_msg_.P[0] = info_msg_.K[0];
    info_msg_.P[5] = info_msg_.K[4];
    info_msg_.P[2] = info_msg_.K[2];
    info_msg_.P[6] = info_msg_.K[5];
    info_msg_.P[10] = info_msg_.K[8];
}


bool TelloROS::takeoffCallback(std_srvs::Empty::Request  &,
                               std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("takeoff");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::landCallback(std_srvs::Empty::Request  &,
                            std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("land");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::flipRCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("flip r");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::flipLCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("flip l");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::flipFCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("flip f");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::flipBCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("flip b");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::hoverCallback(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("stop");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::emergencyCallback(std_srvs::Empty::Request  &,
                                 std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("emergency");
    while (!(tello_ptr_->ReceiveResponse()));
    return true;
}


bool TelloROS::enableStreamCallback(std_srvs::Empty::Request  &,
                                    std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("streamon");
    while (!(tello_ptr_->ReceiveResponse()));
    tello_ptr_->OpenStream();
    camera_timer_.start();
    return true;
}


bool TelloROS::disableStreamCallback(std_srvs::Empty::Request  &,
                                    std_srvs::Empty::Response &)
{
    tello_ptr_->SendCommand("streamoff");
    while (!(tello_ptr_->ReceiveResponse()));
    tello_ptr_->CloseStream();
    camera_timer_.stop();
    return true;
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
    std::map<std::string,std::string> state;
    tello_ptr_->GetState(state);
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


void TelloROS::cameraLoop(const ros::TimerEvent&)
{
    cv::Mat frame;
    tello_ptr_->GetFrame(frame);
    if(frame.empty())
    {
        ROS_ERROR_STREAM("Empty Frame");
    }
    if(!frame.empty())
    {
        sensor_msgs::ImagePtr camera_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        info_msg_.header = camera_msg->header;

        camera_pub_.publish(*camera_msg, info_msg_);
    }
}
} // namespace tello_ros
