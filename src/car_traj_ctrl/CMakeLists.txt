cmake_minimum_required(VERSION 3.0.2)
project(car_traj_ctrl)

# Add package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  project_msgs
)

# Add header files to the project, including the global header rosConfig.hpp.
file(
  GLOB_RECURSE HEADERS #include/*.hpp
  ${PROJECT_SOURCE_DIR}/../rosConfig.hpp
  ${PROJECT_SOURCE_DIR}/../parameters.hpp
  )

# Add source files to the project
file(GLOB_RECURSE SOURCES src/controller.cpp)

# Create a library target for the package
add_library(${PROJECT_NAME}_lib ${SOURCES} ${HEADERS})

# Make sure custom messages are built before compiling library
add_dependencies(${PROJECT_NAME}_lib project_msgs_generate_messages_cpp)

# Set include directories for the package
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    include
    ${PROJECT_SOURCE_DIR}/../
    ${catkin_INCLUDE_DIRS}
  PRIVATE
    src
)

# Set linker to c++
set_target_properties(${PROJECT_NAME}_lib PROPERTIES LINKER_LANGUAGE CXX)

# 
catkin_package(
  INCLUDE_DIRS #include
  CATKIN_DEPENDS project_msgs roscpp #std_msgs rospy 
)

# Add the ROS node executables
add_executable(${PROJECT_NAME}_exec src/controller.cpp)

# Specify that the executables depends on the catkin libraries
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME}_lib ${catkin_LIBRARIES})