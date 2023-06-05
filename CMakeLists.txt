cmake_minimum_required(VERSION 3.5)
project(sbg_simu)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_subdirectory(external/sbgECom)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/sbg_simu/
  ${PROJECT_SOURCE_DIR}/external/sbgECom/common
  ${PROJECT_SOURCE_DIR}/external/sbgECom/src
)

set (SBG_COMMON_RESOURCES
  src/sbg_simu.cpp
)

add_executable(sbg_simu ${SBG_COMMON_RESOURCES} src/main.cpp)

ament_target_dependencies(sbg_simu rclcpp std_msgs sensor_msgs)

target_link_libraries(sbg_simu sbgECom)

install(TARGETS
  sbg_simu
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  PATTERN ".git" EXCLUDE)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".git" EXCLUDE)

ament_package()
