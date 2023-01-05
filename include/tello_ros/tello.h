#ifndef TELLO_H
#define TELLO_H

#include <sys/socket.h>
#include <sys/types.h>

#include <vector>
#include <map>
#include <experimental/optional>

#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// This is the server running in Tello, where we send commands to and we
// receive responses from
const char* const TELLO_SERVER_IP{"192.168.10.1"};
const char* const TELLO_SERVER_COMMAND_PORT{"8889"};
const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
const double TELLO_CAMERA_FOV{1.4409};
const int TELLO_CAMERA_WIDTH{1280};
const int TELLO_CAMERA_HEIGHT{720};

// This is the local port where we bind our local UDP client to.
//
// NOTE 1: With UDP, we have to bind() the socket in the client because UDP is
// connectionless, so there is no other way for the stack to know which
// program to deliver datagrams to for a particular port.
//
// NOTE 2: Tello will respond to any port, but it keeps that port number
// remembered while it's powered up. So you can't have more than one client
// receiving responses from the Tello.
const int LOCAL_CLIENT_COMMAND_PORT{9000};

// We need to start a local UPD server to receive state updates.
const int LOCAL_SERVER_STATE_PORT{8890};

namespace tello_ros
{
class Tello
{
public:
    Tello();
    ~Tello();
    bool Bind(int local_client_command_port = LOCAL_CLIENT_COMMAND_PORT);
    bool SendCommand(const std::string& command);
    std::experimental::optional<std::string> ReceiveResponse();
    void GetState(std::map<std::string,std::string>& tello_stat);
    void GetFrame(cv::Mat& frame);
    void OpenStream();
    void CloseStream();

    Tello(const Tello&) = delete;
    Tello(const Tello&&) = delete;
    Tello& operator=(const Tello&) = delete;
    Tello& operator=(const Tello&&) = delete;

private:
    void FindTello();
    void ShowTelloInfo();

    int m_command_sockfd{0};
    int m_state_sockfd{0};
    int m_local_client_command_port{LOCAL_CLIENT_COMMAND_PORT};
    sockaddr_storage m_tello_server_command_addr{};
    cv::VideoCapture capture_;
};
}  // namespace tello_ros
#endif //TELLO_H
