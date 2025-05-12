cmake_minimum_required(VERSION 3.8)
project(skeleton_image_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# OpenNI2 ve NiTE2 yollarını manuel olarak tanımla
set(OpenNI2_INCLUDE_DIRS /usr/include/openni2)
set(OpenNI2_LIBRARIES /usr/lib/x86_64-linux-gnu/libOpenNI2.so)
set(NiTE2_INCLUDE_DIRS /home/enesisaoglu/NiTE-Linux-x64-2.2/Include)
set(NiTE2_LIBRARIES /home/enesisaoglu/NiTE-Linux-x64-2.2/Redist/libNiTE2.so)

add_executable(skeleton_image_publisher src/skeleton_image_publisher.cpp)
target_include_directories(skeleton_image_publisher PUBLIC
  ${OpenNI2_INCLUDE_DIRS}
  ${NiTE2_INCLUDE_DIRS}
)
target_link_libraries(skeleton_image_publisher
  ${OpenNI2_LIBRARIES}
  ${NiTE2_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(skeleton_image_publisher
  rclcpp sensor_msgs cv_bridge
)

install(TARGETS skeleton_image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()