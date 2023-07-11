#include <fstream>
#include <project_msgs/ControllerInputMessage.h>
#include <project_msgs/SimulatorStateMessage.h>
#include <ros/ros.h>
#include <rosConfig.hpp>
#include <rosbag/bag.h>

const std::string FILENAME = "test.cpp";

// CONFIGURABLE CONSTANTS
const std::string         ROS_BAG_FILE_NAME   = "testBag1.bag";
const std::vector<double> STEP_INPUT_ONE      = {1.0, 0.0};
const std::vector<double> STEP_INPUT_TWO      = {1.0, 0.1};
constexpr double          STEP_ONE_TIME       = 3.0;
constexpr double          STEP_TWO_TIME       = 7.0;
constexpr double          SIMULATION_END_TIME = 10.0;

static_assert(STEP_ONE_TIME < STEP_TWO_TIME);
static_assert(STEP_TWO_TIME < SIMULATION_END_TIME);

// Declare a global variable to store the simulator state, and a mutex to ensure no simultaneous access.
project_msgs::SimulatorStateMessage g_simulatorState;
std::mutex                          stateMutex;

project_msgs::SimulatorStateMessage getState() {
    std::lock_guard<std::mutex> lock(stateMutex);
    return g_simulatorState;
}

// Callback function to receive simulator state messages
void simulatorStateCallback(const project_msgs::SimulatorStateMessage::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(stateMutex);
    g_simulatorState.systemState    = msg->systemState;
    g_simulatorState.simulationTime = msg->simulationTime;
    g_simulatorState.wallClockTime  = msg->wallClockTime;
}

int main(int argc, char *argv[]) {
    // Init ROS node and create node handle
    ros::init(argc, argv, Parameters::RosRelated::TEST_SIMULATOR_ROS_NODE_NAME);
    ros::NodeHandle nh;

    // Create a subscriber and publisher for input and state
    ros::Subscriber stateSub = nh.subscribe(Parameters::RosRelated::SIMULATOR_STATE_TOPIC_NAME, 10, simulatorStateCallback);
    ros::Publisher  inputPub = nh.advertise<project_msgs::ControllerInputMessage>(Parameters::RosRelated::CONTROLLER_INPUT_TOPIC_NAME, 10);

    // Create a ROS bag to store the simulator state and open the bag for writing
    rosbag::Bag       bag;
    std::string       file          = __FILE__;
    const std::string ABSOLUTE_PATH = file.erase(file.size() - FILENAME.size(), FILENAME.size()) + "../../../bagfiles/" + ROS_BAG_FILE_NAME;
    bag.open(ABSOLUTE_PATH, rosbag::bagmode::Write);

    // Create initial (zero) input message and two step input messages
    project_msgs::ControllerInputMessage input;
    project_msgs::ControllerInputMessage stepOne;
    project_msgs::ControllerInputMessage stepTwo;
    input.controllerInput   = {0.0, 0.0};
    stepOne.controllerInput = STEP_INPUT_ONE;
    stepTwo.controllerInput = STEP_INPUT_TWO;

    // Init local simulationState copy
    project_msgs::SimulatorStateMessage simulatorState;
    simulatorState.simulationTime = 0.0;

    // Initialize callback processing
    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    sleep(Parameters::RosRelated::STARTUP_MARGIN_TIME);

    // Run input loop
    while (simulatorState.simulationTime < SIMULATION_END_TIME) {
        // Check the simulator state
        simulatorState = getState();

        // Use step input messages after a few seconds
        if (simulatorState.simulationTime > STEP_TWO_TIME) {
            input = stepTwo;

        } else if (simulatorState.simulationTime > STEP_ONE_TIME) {
            input = stepOne;
        }

        // Avoid writing unnecessary messages before the simulator has started running
        if (simulatorState.simulationTime > 0.0) {
            // Write the current simulator state and input to the ROS bag
            inputPub.publish(input);
            ros::Time time = ros::Time::now();
            bag.write(Parameters::RosRelated::SIMULATOR_STATE_TOPIC_NAME, time, simulatorState);
            bag.write(Parameters::RosRelated::CONTROLLER_INPUT_TOPIC_NAME, time, input);
        }
    }

    // Close the ROS bag
    bag.close();
    ROS_INFO("Exiting Test Node");
    return 0;
}