cmake_minimum_required(VERSION 3.0.2)
project(project_msgs)

# Add package dependencies
find_package(
    catkin 
    REQUIRED COMPONENTS message_generation std_msgs
)

# Add the custom message files to the project
file(GLOB MESSAGE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/msg/*.msg")

add_message_files(
    FILES
    tm.msg
    SimulatorStateMessage.msg
    ControllerInputMessage.msg
)

message("FILES: " ${FILES})

# Generate message headers
generate_messages(
    # LANGUAGES cpp
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp message_runtime
)

# Create a library target for the package
add_library(${PROJECT_NAME} ${MESSAGE_FILES})

# Set C++ standard
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
