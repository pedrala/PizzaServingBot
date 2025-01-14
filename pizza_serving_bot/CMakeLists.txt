cmake_minimum_required(VERSION 3.5)
project(pizza_serving_bot)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 패키지의 종속성 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pizza_order_msgs REQUIRED)
find_package(cpprestsdk REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav2_msgs REQUIRED)

# Qt5 Widgets 종속성 찾기
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# 자동으로 MOC, UIC, RCC 처리 (Qt 사용 시 필요)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# 포함 디렉토리 설정
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# AMR Controller 노드
add_executable(amr_controller_node
  pizza_serving_bot/amr_controller/amr_node.cpp
)

ament_target_dependencies(amr_controller_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  OpenCV
)

target_include_directories(amr_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Kitchen Monitor 노드
add_executable(kitchen_monitor_node
  pizza_serving_bot/kitchen_monitor/kitchen_node.cpp
)

ament_target_dependencies(kitchen_monitor_node
  rclcpp
  std_msgs
  geometry_msgs
  pizza_order_msgs
)

target_link_libraries(kitchen_monitor_node
  Qt5::Widgets
  cpprest
)

target_include_directories(kitchen_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Table Order 노드
add_executable(table_order_node
  pizza_serving_bot/table_order/kiosk_node.cpp
)

ament_target_dependencies(table_order_node
  rclcpp
  std_msgs
  geometry_msgs
  pizza_order_msgs
)

target_link_libraries(table_order_node
  Qt5::Widgets
)

target_include_directories(table_order_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# 설치 설정
install(TARGETS
  amr_controller_node
  kitchen_monitor_node
  table_order_node
  DESTINATION lib/${PROJECT_NAME}
)

# 런치 파일 설치 (필요한 경우)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
