#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "yolo_cpp_custom_msg/msg/vision.hpp"

using std::placeholders::_1;

int bounding_box = 0;
int titik_tengah = 0;
int fps = 0;

void vision_callback(const yolo_cpp_custom_msg::msg::Vision::SharedPtr msg)
{
  bounding_box = msg->bounding_box;
  titik_tengah = msg->titik_tengah;
  fps = msg->fps;

  std::cout << "Bounding Box: " << bounding_box << std::endl;
  std::cout << "Titik Tengah: " << titik_tengah << std::endl;
  std::cout << "FPS         : " << fps << std::endl;
  std::cout << "-----------------------------" << std::endl;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("vision_sub_node");
  auto subscription = node->create_subscription<yolo_cpp_custom_msg::msg::Vision>(
    "vision_topic", 10, vision_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
