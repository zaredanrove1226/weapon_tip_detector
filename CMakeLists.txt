cmake_minimum_required(VERSION 3.8)

project(weapon_tip_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(current_tip_detector_node_refactored
  src/current_tip_detector_node_refactored.cpp
  src/detector_utils.cpp
  src/depth_projector.cpp
  src/detector_profiles.cpp
  src/detection_pipeline.cpp
  src/preview_debugger.cpp
  src/palm_reference_detector.cpp
)

ament_target_dependencies(current_tip_detector_node_refactored
  rclcpp
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  realsense2_camera_msgs
)

target_link_libraries(current_tip_detector_node_refactored
  ${OpenCV_LIBRARIES}
)

target_include_directories(current_tip_detector_node_refactored PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS current_tip_detector_node_refactored
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint, only works in a git repo
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()