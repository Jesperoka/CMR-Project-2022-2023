#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "ros/ros.h"

#include <parameters.hpp>
#include <project_msgs/ControllerInputMessage.h>
#include <project_msgs/SimulatorStateMessage.h>
#include <rosConfig.hpp>

template <class Stepper, int ID> class Simulator {
  private:
    ros::NodeHandle        nh; // extern? if we want all static template class
    static ros::Subscriber inputSubscriber;
    static ros::Publisher  simulatorStatePublisher;

    static Stepper                                     stepper;
    static std::mutex                                  inputMutex;
    static Parameters::Simulator::InputType            input;
    static Parameters::Simulator::ModelFunctionPointer model;
    static Parameters::Simulator::StateType            doStep(Parameters::Simulator::StateType &state, double &time);

    static void inputCallback(const project_msgs::ControllerInputMessage::ConstPtr &msg);

  public:
    static void                             run(void);
    static Parameters::Simulator::InputType getInput();
    // static void setInput(Parameters::Simulator::InputType);

    // Constructor
    Simulator(const Parameters::Simulator::ModelFunctionPointer &modelPtr) {
        // Simulator stuff
        input = Parameters::Simulator::INITIAL_INPUT;
        model = modelPtr;
        // ROS Stuff
        nh                      = ros::NodeHandle(); // needed?
        simulatorStatePublisher = nh.advertise<project_msgs::SimulatorStateMessage>(Parameters::RosRelated::SIMULATOR_STATE_TOPIC_NAME, 1);
        inputSubscriber         = nh.subscribe(Parameters::RosRelated::CONTROLLER_INPUT_TOPIC_NAME, 100, &inputCallback);
    }
};

// Global scope definition of static members
#define TEMPLATE template <class Stepper, int ID>
// TEMPLATE ros::NodeHandle Simulator<Stepper, ID>::nh;
TEMPLATE ros::Subscriber Simulator<Stepper, ID>::inputSubscriber;
TEMPLATE ros::Publisher Simulator<Stepper, ID>::simulatorStatePublisher;
TEMPLATE Stepper        Simulator<Stepper, ID>::stepper;
TEMPLATE std::mutex Simulator<Stepper, ID>::inputMutex;
TEMPLATE Parameters::Simulator::InputType Simulator<Stepper, ID>::input; // <---
TEMPLATE Parameters::Simulator::ModelFunctionPointer Simulator<Stepper, ID>::model;
#undef TEMPLATE

#endif /* SIMULATOR_HPP */