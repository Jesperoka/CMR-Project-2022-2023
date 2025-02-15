cmake_minimum_required(VERSION 3.0.2)
project(car_simulator)

# Add package dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  # rospy # see if we need this
  # std_msgs # see if we need this
  project_msgs
  # Boost REQUIRED COMPONENTS numeric variant # see if we need this
)

# Add header files to the project, including the global header rosConfig.hpp.
file(
  GLOB_RECURSE HEADERS include/*.hpp
  ${PROJECT_SOURCE_DIR}/../rosConfig.hpp
  ${PROJECT_SOURCE_DIR}/../parameters.hpp
  )

# Add source files to the project
file(GLOB_RECURSE SOURCES src/simulator.cpp)

# Create a library target for the package, with the specified header and source files. 
# Global header rosConfig.hpp is also added.
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS project_msgs roscpp #std_msgs rospy 
)

# Add the ROS node executables
add_executable(${PROJECT_NAME}_exec src/simulator.cpp)

# Specify that the executables depends on the car_simulator library and the catkin dependencies
target_link_libraries(${PROJECT_NAME}_exec ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

# Correctly build the test node as well:
add_executable(test_${PROJECT_NAME}_exec src/test.cpp)
target_link_libraries(test_${PROJECT_NAME}_exec ${PROJECT_NAME}_lib ${catkin_LIBRARIES})