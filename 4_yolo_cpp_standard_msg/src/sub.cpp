#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class VisionSubscriber : public rclcpp::Node
{
public:
  VisionSubscriber()
  : Node("vision_sub_node")
  {
    sub_bbox_ = this->create_subscription<std_msgs::msg::Int32>(
      "bounding_box_topic", 10, std::bind(&VisionSubscriber::bbox_callback, this, _1));

    sub_center_ = this->create_subscription<std_msgs::msg::Int32>(
      "center_topic", 10, std::bind(&VisionSubscriber::center_callback, this, _1));

    sub_fps_ = this->create_subscription<std_msgs::msg::Int32>(
      "fps_topic", 10, std::bind(&VisionSubscriber::fps_callback, this, _1));
  }

private:
  void bbox_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::cout << "Bounding Box : " << msg->data << std::endl;
  }

  void center_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::cout << "Titik Tengah : " << msg->data << std::endl;
  }

  void fps_callback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    std::cout << "FPS          : " << msg->data << std::endl;
    std::cout << "-----------------------------" << std::endl;
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_bbox_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_center_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_fps_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionSubscriber>());
  rclcpp::shutdown();
  return 0;
}
