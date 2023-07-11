#ifndef ROS_CONFIG_HPP
#define ROS_CONFIG_HPP

// Using ROS parameter server is overkill for this project.
namespace Parameters {

namespace RosRelated {
// Nodes
const char SIMULATOR_ROS_NODE_NAME[]      = "car_simulator";
const char TEST_SIMULATOR_ROS_NODE_NAME[] = "test_car_simulator";
const char CONTROLLER_ROS_NODE_NAME[]     = "car_traj_ctrl";
// Topics
const std::string SIMULATOR_STATE_TOPIC_NAME  = "/simulator_state";
const std::string CONTROLLER_INPUT_TOPIC_NAME = "/controller_input";
// Other
const int STARTUP_MARGIN_TIME = 5; // seconds

}; // namespace RosRelated

}; // namespace Parameters

#endif // ROS_CONFIG_HPP