cmake_minimum_required(VERSION 3.0.2)
project(calibrate_lidar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)


generate_messages(DEPENDENCIES sensor_msgs)
catkin_package()

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/calibrate_lidar.cpp
# )

add_executable(Clustering src/clustering.cpp)

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(Clustering ${catkin_LIBRARIES})

