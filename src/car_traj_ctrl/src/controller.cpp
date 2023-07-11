#include <fstream>
#include <parameters.hpp>
#include <project_msgs/ControllerInputMessage.h>
#include <project_msgs/SimulatorStateMessage.h>
#include <ros/ros.h>
#include <rosConfig.hpp>
#include <rosbag/bag.h>
#include <std_msgs/Float64MultiArray.h>

const std::string FILENAME          = "controller.cpp";
const std::string ROS_BAG_FILE_NAME = "trajectoryTrack.bag";
const int         YAW_INDEX         = Parameters::Simulator::MODEL == Parameters::Simulator::CompatibleModels::BICYCLE_REAR_DRIVE_KINEMATIC ? 2 : 4;

// Declare a global variable to store the simulator state, and a mutex to ensure no simultaneous access.
project_msgs::SimulatorStateMessage g_simulatorState;
bool                                g_receivedFirstMessage = false;
std::mutex                          stateMutex;

project_msgs::SimulatorStateMessage getState() {
    std::lock_guard<std::mutex> lock(stateMutex);
    return g_simulatorState;
}

// Callback function to receive simulator state messages
void simulatorStateCallback(const project_msgs::SimulatorStateMessage::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(stateMutex);
    g_receivedFirstMessage          = true;
    g_simulatorState.systemState    = msg->systemState;
    g_simulatorState.simulationTime = msg->simulationTime;
    g_simulatorState.wallClockTime  = msg->wallClockTime;
}

//
int main(int argc, char *argv[]) {
    // Init ROS node and create node handle
    ros::init(argc, argv, Parameters::RosRelated::TEST_SIMULATOR_ROS_NODE_NAME);
    ros::NodeHandle nh;

    // Initialize callback processing
    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    // Create a subscriber and publisher for input and state
    ros::Subscriber stateSub = nh.subscribe(Parameters::RosRelated::SIMULATOR_STATE_TOPIC_NAME, 10, simulatorStateCallback);
    ros::Publisher  inputPub = nh.advertise<project_msgs::ControllerInputMessage>(Parameters::RosRelated::CONTROLLER_INPUT_TOPIC_NAME, 10);

    // Create a ROS bag to store the simulator state and open the bag for writing
    rosbag::Bag       bag;
    std::string       file          = __FILE__;
    const std::string ABSOLUTE_PATH = file.erase(file.size() - FILENAME.size(), FILENAME.size()) + "../../../bagfiles/" + ROS_BAG_FILE_NAME;
    bag.open(ABSOLUTE_PATH, rosbag::bagmode::Write);

    // Initialize local simulationState copy
    project_msgs::SimulatorStateMessage simulatorStateMessage;
    simulatorStateMessage.simulationTime = 0.0;
    double t                             = simulatorStateMessage.simulationTime;

    // Initialize controller input message
    project_msgs::ControllerInputMessage inputMessage;
    inputMessage.controllerInput = {0.0, 0.0};

    // Trajectory parameters
    double T        = Parameters::ReferenceTrajectory::SINUSOIDAL_PERIOD;
    double a        = Parameters::ReferenceTrajectory::SINUSOIDAL_AMPLITUDE;
    double reducedT = (1 - Parameters::ReferenceTrajectory::SINUSOIDAL_PERIOD_REDUCTION_FRACTION) * T;

    // Robot length
    const double L = Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_FRONT_AXLE + Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_REAR_AXLE;

    // Dummy message to store reference trajectory in rosbag
    std_msgs::Float64MultiArray referenceTrajectory;

    // Wait for the simulator to publish state
    while (!g_receivedFirstMessage) {
        usleep(100);
    }

    // Run input loop
    while (simulatorStateMessage.simulationTime < Parameters::Controller::SHUT_OFF_TIME) {
        // Check the simulator state
        simulatorStateMessage = getState();

        double t   = simulatorStateMessage.simulationTime;
        double x   = simulatorStateMessage.systemState[0];
        double y   = simulatorStateMessage.systemState[1];
        double yaw = simulatorStateMessage.systemState[YAW_INDEX];

        // Trajectory sinusoid angular frequency
        double twoPiOverT = t >= Parameters::ReferenceTrajectory::SINUSOIDAL_PERIOD_REDUCTION_TIME ? (2 * M_PI / reducedT) : (2 * M_PI / T);
        // Trajectory reference point
        double xp = a * std::sin(twoPiOverT * t);
        double yp = a * std::sin(twoPiOverT * t) * std::cos(twoPiOverT * t);
        // Reference point first time derivative
        double vxp = twoPiOverT * a * std::cos(twoPiOverT * t);
        double vyp = twoPiOverT * a * (std::pow(std::cos(twoPiOverT * t), 2.0) - std::pow(std::sin(twoPiOverT * t), 2.0));
        // Reference point second time derivative
        double axp = -std::pow(twoPiOverT, 2.0) * a * std::sin(twoPiOverT * t);
        double ayp = -4.0 * std::pow(twoPiOverT, 2.0) * a * std::sin(twoPiOverT * t) * std::cos(twoPiOverT * t);
        // Distance to reference point
        double epsilon = (xp - x) + (yp - y);

        // Feedback linearization transformation
        double v        = vxp * std::cos(yaw) + vyp * std::sin(yaw);
        double nonZeroV = v == 0.0 ? v + 1e-5 : v;
        double phiRef   = std::atan((L / epsilon) * (vyp * std::cos(yaw) - vxp * std::sin(yaw)) / (nonZeroV));

        // double omega    = (1/(std::pow(std::cos(), 2))*;
        // double omega = std::min((1 / epsilon), Parameters::Controller::MAX_OMEGA) * (vyp * std::cos(yaw) - vxp * std::sin(yaw));

        inputMessage.controllerInput[0] = v;
        inputMessage.controllerInput[1] = omega;
        referenceTrajectory.data        = {xp, yp, vxp, vyp};

        // Avoid writing unnecessary messages before the simulator has started running
        if (simulatorStateMessage.simulationTime > 0.0) {
            // Write the current simulator state and input to the ROS bag
            inputPub.publish(inputMessage);
            ros::Time time = ros::Time::now();
            bag.write(Parameters::RosRelated::SIMULATOR_STATE_TOPIC_NAME, time, simulatorStateMessage);
            bag.write(Parameters::RosRelated::CONTROLLER_INPUT_TOPIC_NAME, time, inputMessage);
            bag.write("/FAKE_TOPIC", time, referenceTrajectory);
        }
    }

    // Close the ROS bag
    bag.close();
    ROS_INFO("Exiting Controller Node");
    return 0;
}