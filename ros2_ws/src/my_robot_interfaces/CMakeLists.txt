cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 모든 인터페이스 등록 (msg + srv + action)
# rosidl_generate_interfaces: .msg/.srv/.action → Python/C++ 코드 자동 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectionArray.msg"
  "srv/NavigateToObject.srv"
  "action/MoveToTarget.action"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_package()
