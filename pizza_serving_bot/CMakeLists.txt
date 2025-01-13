cmake_minimum_required(VERSION 3.5)
project(pizza_serving_bot)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 의존성 추가
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(Qt5Widgets REQUIRED)  # GUI를 위한 Qt

# 실행 파일 생성
add_executable(amr_node src/amr_node.cpp)
ament_target_dependencies(amr_node rclcpp nav_msgs geometry_msgs nav2_msgs Qt5::Widgets)

add_executable(kitchen_node src/kitchen_node.cpp)
ament_target_dependencies(kitchen_node rclcpp)

add_executable(kiosk_node src/kiosk_node.cpp)
ament_target_dependencies(kiosk_node rclcpp)

# 설치 경로 설정
install(TARGETS
  amr_node
  kitchen_node
  kiosk_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
