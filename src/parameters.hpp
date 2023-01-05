#ifndef PARAMETERS_HPP
#define PARAMETERS_HPP

#include <vector>

// Using ROS parameter server is overkill for this project.
namespace Parameters {

// TODO: description
namespace Simulator {
// Aliases
using StateType            = std::vector<double>;
using InputType            = std::vector<double>;
using ModelFunctionPointer = void (*)(const StateType &, StateType &, const double);
// Values
const StateType INITIAL_STATE            = {0.0, 0.0, 0.0, 0.0};
const InputType INITIAL_INPUT            = {0.0, 0.0};
const double    INITIAL_SIMULATION_TIME  = 0.0;
const double    INITIAL_STEP_SIZE        = 0.01;
const double    SLEEP_SECONDS            = 0.5 * INITIAL_STEP_SIZE;
const int       MAX_STEP_SIZE_REDUCTIONS = 100;

// TODO: description
enum CompatibleModels {
    BICYCLE_REAR_DRIVE_SINGLE_TRACK_DYNAMIC_LINEAR_TIRE,
    BICYCLE_REAR_DRIVE_SINGLE_TRACK_DYNAMIC_FIALA_TIRE,
    BICYCLE_REAR_DRIVE_KINEMATIC,
};
const int MODEL = BICYCLE_REAR_DRIVE_KINEMATIC; // CHOOSE SIMULATION MODEL HERE

// TODO: description
enum CompatibleSteppers {
    // ERK1_EULER,
    // ERK2_MIDPOINT,
    // ERK4_CLASSIC,
    // Adaptive
    ADAPTIVE_ERK45_CASH_KARP,
    ADAPTIVE_ERK45_DORMAND_PRINCE,
    ADAPTIVE_ERK78_FEHLBERG
};
const int STEPPER = ADAPTIVE_ERK45_DORMAND_PRINCE; // CHOOSE INTEGRATION METHOD HERE
};                                                 // namespace Simulator

// TODO: description
namespace Robot {
const double TOTAL_MASS                                    = 3.2;   // kg
const double DISTANCE_FROM_CENTER_OF_GRAVITY_TO_FRONT_AXLE = 0.14;  // m
const double DISTANCE_FROM_CENTER_OF_GRAVITY_TO_REAR_AXLE  = 0.12;  // m
const double GROUND_FRICTION_COEFFICIENT                   = 0.385; // dimensionless
const double FRONT_WHEEL_CORNERING_STIFFNESS               = 50;    // N/rad
const double REAR_WHEEL_CORNERING_STIFFNESS                = 120;   // N/rad
const double YAW_INERTIA                                   = 0.010; // kg m²
};                                                                  // namespace Robot

// TODO: description
namespace Controller {
// Tuning Parameters
const double PROPORTIONAL_GAIN_X      = 1;
const double PROPORTIONAL_GAIN_Y      = 1;
const double INTEGRAL_TIME_CONSTANT_X = 1;
const double INTEGRAL_TIME_CONSTANT_Y = 1;
const double SAMPLING_TIME            = 0.1;
// Convenience Parameters
const double SHUT_OFF_TIME = 15.0;
const double MAX_OMEGA     = M_PI * 10; // rad/s
} // namespace Controller

// TODO: description
namespace ReferenceTrajectory {
const double SINUSOIDAL_PERIOD                    = 2.5; // s
const double SINUSOIDAL_AMPLITUDE                 = 2;   // m
const double SINUSOIDAL_PERIOD_REDUCTION_FRACTION = 0.25;
const double SINUSOIDAL_PERIOD_REDUCTION_TIME     = Controller::SHUT_OFF_TIME;
}; // namespace ReferenceTrajectory

}; // end namespace Parameters

#endif // PARAMETERS_HPP