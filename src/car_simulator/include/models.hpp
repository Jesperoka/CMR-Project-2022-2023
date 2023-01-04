#ifndef MODELS_HPP
#define MODELS_HPP

#include <cmath>
#include <parameters.hpp>
#include <vector>

// TODO: description
namespace Models {

// Constants for readability
const double L = Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_FRONT_AXLE + Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_REAR_AXLE;

/**
 * @brief TODO
 *
 * @param x
 * @param dxdt
 */
template <std::vector<double> (*getInput)(void)>
void bicycleRearDriveFrontSteerSingleTrackDynamic(const std::vector<double> &x, std::vector<double> &dxdt, const double t) {
    std::vector<double> input = getInput();
    dxdt[0]                   = 1;
    dxdt[1]                   = 1;
    dxdt[2]                   = 1;
    dxdt[3]                   = 1;
}

/**
 * @brief
 *
 * @tparam getInputs
 * @param x
 * @param dxdt
 * @param t
 */
template <std::vector<double> (*getInput)(void)>
void bicycleRearDriveFrontSteerKinematic(const std::vector<double> &x, std::vector<double> &dxdt, const double t) {
    std::vector<double> input = (*getInput)();
    dxdt[0]                   = input[0] * std::cos(x[2]);
    dxdt[1]                   = input[0] * std::sin(x[2]);
    dxdt[2]                   = input[0] * std::tan(x[3]) / L;
    dxdt[3]                   = input[1];
}

} // namespace Models

#endif // MODELS_HPP