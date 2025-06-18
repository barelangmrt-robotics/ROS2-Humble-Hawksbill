cmake_minimum_required(VERSION 3.8)
project(yolo_python_custom_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cari dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # buat dependencies custom message
find_package(rosidl_typesupport_c REQUIRED) # buat dependencies custom message

# buat custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vision.msg" # custom message harus diawali huruf kapital
  DEPENDENCIES std_msgs
)

# daftarkan file source untuk python
install(PROGRAMS
  src/pub.py # file python (publisher)
  src/sub.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
