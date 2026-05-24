cmake_minimum_required(VERSION 3.8)
project(amr_sweeper_default_missions)

find_package(ament_cmake REQUIRED)

install(DIRECTORY missions
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
