cmake_minimum_required(VERSION 3.8)
project(rosbag_image_save)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)  # 添加 geometry_msgs
find_package(jsoncpp REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(${PROJECT_NAME}_node src/my_rosbag.cpp)
add_executable(${PROJECT_NAME}_slam_node src/my_rosbag_SLAM.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs  # 添加 geometry_msgs
  jsoncpp
  OpenCV 
  message_filters  # 添加message_filters到链接依赖
)

ament_target_dependencies(${PROJECT_NAME}_slam_node
  rclcpp
  sensor_msgs
  cv_bridge
  geometry_msgs  # 添加 geometry_msgs
  jsoncpp
  OpenCV 
  nav_msgs
  message_filters  # 添加message_filters到链接依赖
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
