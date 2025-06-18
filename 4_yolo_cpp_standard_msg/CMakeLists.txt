cmake_minimum_required(VERSION 3.8)
project(yolo_cpp_standard_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# cari dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# buat file executable untuk sub cpp
add_executable(sub_node src/sub.cpp) # setiap jalankan file sub.cpp, gunakan sub_node
ament_target_dependencies(sub_node rclcpp std_msgs)

# daftarkan file source untuk cpp
install(TARGETS
  sub_node # file cpp (subscriber)
  DESTINATION lib/${PROJECT_NAME}
)

# daftarkan file source untuk python
install(PROGRAMS
  src/pub.py # file python (publisher)
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
