#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <boost/asio.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

using namespace std;

#define HEADER(x)	(x == 0xFB)
#define HEAD_ID(x)	(x >= 0x01 && x <= 0x01)
#define POSE_ID(x)	(x == 0x01)

class opti_serial_pub : public rclcpp::Node
{
  public:
    opti_serial_pub();
    ~opti_serial_pub();

  private:
    void timer_callback();
    void EnQueue(uint8_t);
    void Decode_poses(vector<uint8_t>);
    boost::asio::io_service ios;
    boost::asio::serial_port port;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr poses_pub;
    size_t count_;

    uint8_t my_id;
    string topic_name;
    bool f_poses;
    string serial_port;
    uint16_t serial_buad;

    uint8_t buffer[512];
    vector<uint8_t> packet_buf;
    uint8_t packet_id;
    uint8_t packet_size;
};