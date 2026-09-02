cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core features2d imgproc video)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(odometry_projection_node
  src/odometry_projection_node.cpp
)
target_compile_features(odometry_projection_node PUBLIC cxx_std_17)
target_include_directories(odometry_projection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(odometry_projection_node
  geometry_msgs
  nav_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(visual_odometry_node
  src/visual_odometry_node.cpp
)
target_compile_features(visual_odometry_node PUBLIC cxx_std_17)
target_include_directories(visual_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(visual_odometry_node
  ${OpenCV_LIBS}
)
ament_target_dependencies(visual_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(
  TARGETS
    odometry_projection_node
    visual_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    ../../tests/vo_localization_benchmark/localization_gnss_benchmark.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  ../../tests/vo_localization_benchmark/analyze_localization_benchmark.py
  ../../tests/vo_localization_benchmark/localization_benchmark_driver.py
  ../../tests/vo_localization_benchmark/run_localization_gnss_benchmark.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()
