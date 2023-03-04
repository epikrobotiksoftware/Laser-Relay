#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <rmw/qos_profiles.h>
#include <rclcpp/qos.hpp>

using std::placeholders::_1;

class LaserRelay : public rclcpp::Node
{
public:
    LaserRelay()
        : Node("laser_relay"), laser_data_(nullptr)
    {
        this->declare_parameter("topic_in", "topic_in1");
        this->declare_parameter("topic_out", "topic_out");

        std::string topic_in =
            this->get_parameter("topic_in").get_parameter_value().get<std::string>();

        std::string topic_out =
            this->get_parameter("topic_out").get_parameter_value().get<std::string>();

        RCLCPP_INFO(this->get_logger(), "Relaying from %s to %s!", topic_in.c_str(), topic_out.c_str());

        // Set up QoS profile
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data);

        // Create subscription and publisher
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            topic_in, qos, std::bind(&LaserRelay::listener_callback, this, _1));
        relayed_topic_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(topic_out, 10);
    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_data_ = msg;
        relayed_topic_pub_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr relayed_topic_pub_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto laser_relay_node = std::make_shared<LaserRelay>();
    rclcpp::spin(laser_relay_node);
    rclcpp::shutdown();
    return 0;
}
