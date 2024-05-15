cmake_minimum_required(VERSION 3.8)
project(traversability_analysis)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(ament_cmake REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPENDENCIES
  grid_map_core
  grid_map_ros
  grid_map_msgs
  PCL
  pcl_msgs
  OpenCV
  rclcpp
  Eigen
  message_filters
  sensor_msgs
  geometry_msgs
  nav2_msgs
  tf2 
  tf2_ros 
  tf2_eigen 
  tf2_geometry_msgs
)



include_directories(
  include
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


##############
# Executable #
##############

# add_library(${PROJECT_NAME}_library
# src/traversabilityAnalysis.cpp
# )

# ament_target_dependencies(${PROJECT_NAME}_library
#   ${DEPENDENCIES}
# )

add_executable(${PROJECT_NAME}
  src/traversability_analysis_node.cpp src/traversabilityAnalysis.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}
  ${DEPENDENCIES}
)
if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

#############
## Install ##
#############


install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)


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


