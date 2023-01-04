#include <boost/numeric/odeint.hpp>
#include <boost/variant.hpp>
#include <chrono>
#include <models.hpp>
#include <simulator.hpp>

// Local convenience functions
namespace {
// Assignment helper because ROS messages with many fields are a pain in the ass to assign to.
project_msgs::tm &assign(project_msgs::tm &tmMsg, const std::chrono::time_point<std::chrono::system_clock> &timeNow) {
    //
    std::tm tmBuiltIn{};
    time_t   secondsSince1970 = std::chrono::system_clock::to_time_t(timeNow);
    std::tm *tmPtr            = localtime_r(&secondsSince1970, &tmBuiltIn);

    // Set the values of the tmMsg fields based on the values in the tm struct
    tmMsg.tm_sec   = tmPtr->tm_sec;
    tmMsg.tm_min   = tmPtr->tm_min;
    tmMsg.tm_hour  = tmPtr->tm_hour;
    tmMsg.tm_mday  = tmPtr->tm_mday;
    tmMsg.tm_mon   = tmPtr->tm_mon;
    tmMsg.tm_year  = tmPtr->tm_year;
    tmMsg.tm_wday  = tmPtr->tm_wday;
    tmMsg.tm_yday  = tmPtr->tm_yday;
    tmMsg.tm_isdst = tmPtr->tm_isdst;

    // Return a reference to the tm object
    return tmMsg;
}

template <typename SimulatorType> // const Parameters::Simulator::ModelFunctionPointer
const Parameters::Simulator::ModelFunctionPointer selectModel(const int MODEL) {
    using namespace Parameters::Simulator;

    // constexpr Parameters::Simulator::InputType(*modelFunctionPointer)() = &SimulatorType::getInput;

    switch (MODEL) {
    case CompatibleModels::BICYCLE_REAR_DRIVE_SINGLE_TRACK_DYNAMIC:
        return &Models::bicycleRearDriveFrontSteerSingleTrackDynamic<SimulatorType::getInput>;

    case CompatibleModels::BICYCLE_REAR_DRIVE_KINEMATIC:
        return &Models::bicycleRearDriveFrontSteerKinematic<SimulatorType::getInput>;

    default:
        ROS_ERROR("!!!!!");
        return Parameters::Simulator::ModelFunctionPointer{};
    }
}

template <int ID> auto createSimulator(const int MODEL, const int STEPPER) {

    using namespace boost::numeric::odeint;
    using namespace Parameters::Simulator;

    using Stepper1       = controlled_runge_kutta<runge_kutta_cash_karp54<StateType>>;
    using Stepper2       = controlled_runge_kutta<runge_kutta_dopri5<StateType>>;
    using Stepper3       = controlled_runge_kutta<runge_kutta_fehlberg78<StateType>>;
    using DefaultStepper = Stepper1;

    using SimulatorVariant = boost::variant<Simulator<Stepper1, ID>, Simulator<Stepper2, ID>, Simulator<Stepper3, ID>, Simulator<DefaultStepper, ID>>;

    switch (STEPPER) {
    case CompatibleSteppers::ADAPTIVE_ERK45_CASH_KARP:
        return SimulatorVariant(Simulator<Stepper1, ID>(selectModel<Simulator<Stepper1, ID>>(MODEL)));

    case CompatibleSteppers::ADAPTIVE_ERK45_DORMAND_PRINCE:
        return SimulatorVariant(Simulator<Stepper2, ID>(selectModel<Simulator<Stepper2, ID>>(MODEL)));

    case CompatibleSteppers::ADAPTIVE_ERK78_FEHLBERG:
        return SimulatorVariant(Simulator<Stepper3, ID>(selectModel<Simulator<Stepper3, ID>>(MODEL)));

    default:
        throw std::runtime_error("Incompatible stepper type.");
    }
}

} // end anonymous namespace

/**
 * @brief
 *
 * @tparam Stepper
 * @tparam ID
 * @return Parameters::Simulator::InputType
 */
template <class Stepper, int ID> Parameters::Simulator::InputType Simulator<Stepper, ID>::getInput() {
    std::lock_guard<std::mutex> lock(inputMutex);
    return Simulator<Stepper, ID>::input;
}

/**
 * @brief
 *
 * @tparam Stepper
 * @tparam ID
 * @param msg
 */
template <class Stepper, int ID> void Simulator<Stepper, ID>::inputCallback(const project_msgs::ControllerInputMessage::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(inputMutex);
    input = msg->controllerInput;
}

/**
 * @brief
 *
 * @tparam Stepper
 * @tparam ID
 * @param state
 * @param t
 * @return Parameters::Simulator::StateType
 */
template <class Stepper, int ID> Parameters::Simulator::StateType Simulator<Stepper, ID>::doStep(Parameters::Simulator::StateType &state, double &t) {
    const int N  = Parameters::Simulator::MAX_STEP_SIZE_REDUCTIONS;
    double    dt = Parameters::Simulator::INITIAL_STEP_SIZE;
    int       i  = 0;

    while (i < N) // TODO: implement so both adaptive and fixed steppers are usable.
    {
        if (stepper.try_step(model, state, t, dt) == boost::numeric::odeint::controlled_step_result::fail) {
            i++;
        } else {
            t += dt;
            return state;
        }
    }
    throw std::runtime_error("Did not meet error tolerances after reducing step size" + std::to_string(N) + "times.");
}

/**
 * @brief
 *
 * @tparam Stepper
 * @tparam ID
 */
template <class Stepper, int ID> void Simulator<Stepper, ID>::run() {
    // Initialize callback processing
    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();
    sleep(1.0);

    // Initialize the simulator state
    project_msgs::SimulatorStateMessage simulatorStateMessage;
    simulatorStateMessage.systemState    = Parameters::Simulator::INITIAL_STATE;
    simulatorStateMessage.simulationTime = Parameters::Simulator::INITIAL_SIMULATION_TIME;
    simulatorStateMessage.wallClockTime  = assign(simulatorStateMessage.wallClockTime, std::chrono::system_clock::now());

    while (ros::ok()) {
        // Publish state of simulation to rostopic
        simulatorStatePublisher.publish(simulatorStateMessage);

        // Do simulation step
        simulatorStateMessage.systemState   = doStep(simulatorStateMessage.systemState, simulatorStateMessage.simulationTime);
        simulatorStateMessage.wallClockTime = assign(simulatorStateMessage.wallClockTime, std::chrono::system_clock::now());

        // Approximate real time system evolution. Allows a controller to also have a delay.
        usleep(Parameters::Simulator::SIMULATION_SLEEP_SECONDS * 1e6);
    }
}

int main(int argc, char *argv[]) {
    // Init ROS node
    ros::init(argc, argv, Parameters::RosRelated::SIMULATOR_ROS_NODE_NAME);

    // Every Simulator instance needs a unique ID, because I got lost in the sauce with templates,
    // and trying to seperate Simulator from model implementation. Doesn't matter though, since we only need one
    // simulator anyway.
    const int SIMULATOR_ID = 0;

    // Convenience function to create the simulator based on values in config file. Allows for different model functions
    // and steppers (if implemented).
    auto simulator = createSimulator<SIMULATOR_ID>(Parameters::Simulator::MODEL, Parameters::Simulator::STEPPER);

    // Run the simulation. Works for all Simulators with compatible steppers and models, i.e. SimulatorVariants.
    boost::apply_visitor([](auto &sim) { sim.run(); }, simulator);

    // Don't exit the program.
    ros::spin();
}