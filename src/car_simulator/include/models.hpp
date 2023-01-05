#ifndef MODELS_HPP
#define MODELS_HPP

#include <cmath>
#include <parameters.hpp>
#include <vector>

// TODO: description
namespace Models {

// Constants for readability
const double M   = Parameters::Robot::TOTAL_MASS;
const double a   = Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_FRONT_AXLE;
const double b   = Parameters::Robot::DISTANCE_FROM_CENTER_OF_GRAVITY_TO_REAR_AXLE;
const double L   = a + b;
const double mu  = Parameters::Robot::GROUND_FRICTION_COEFFICIENT;
const double CaF = Parameters::Robot::FRONT_WHEEL_CORNERING_STIFFNESS;
const double CaR = Parameters::Robot::REAR_WHEEL_CORNERING_STIFFNESS;
const double Iz  = Parameters::Robot::YAW_INERTIA;
const double g   = 9.80665;

// Sign(x) function, taken from https://stackoverflow.com/a/4609795
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * @brief TODO
 *
 * @param x state vector (x y beta r psi)
 * @param dxdt time derivative of state vector
 */
template <std::vector<double> (*getInput)(void)>
void bicycleRearDriveFrontSteerSingleTrackDynamicLinearTire(const std::vector<double> &x, std::vector<double> &dxdt, const double t) {
    std::vector<double> input = getInput();

    // Wheel slip angles (as opposed to car/robot slip angle)
    double alphaF = x[2] + a * x[3] / input[0] - input[1];
    double alphaR = x[2] - b * x[3] / input[0];

    // Linear tire model lateral forces
    double Fyf = -CaF * alphaF;
    double Fyr = -CaR * alphaR;

    // Absolute velocity, V = Vx/cos(beta)
    double V = input[0] / std::cos(x[2]);

    // Dynamic model (linear tire model doesnt consider longitudinal tire forces)
    dxdt[0] = V * std::cos(x[4] + x[2]);
    dxdt[1] = V * std::sin(x[5] + x[2]);
    dxdt[2] = (Fyf * std::cos(input[1] - x[2]) + Fyr * std::cos(x[2]) - M * x[3] * V) / (M * V);
    dxdt[3] = (a * Fyf * std::cos(input[1]) - b * Fyr) / Iz;
    dxdt[4] = x[3];
}

/**
 * @brief TODO
 *
 * @param x state vector (x y beta r psi)
 * @param dxdt time derivative of state vector
 */
template <std::vector<double> (*getInput)(void)>
void bicycleRearDriveFrontSteerSingleTrackDynamicFialaTire(const std::vector<double> &x, std::vector<double> &dxdt, const double t) {
    std::vector<double> input = getInput();

    // Wheel slip angles (as opposed to car/robot slip angle)
    double alphaF = x[2] + a * x[3] / input[0] - x[5];
    double alphaR = x[2] - b * x[3] / input[0];

    // Fiala tire model variables
    double Fzf  = M * g * b / (a + b);
    double Fzr  = M * g * a / (a + b);
    double zF   = std::tan(alphaF);
    double zR   = std::tan(alphaR);
    double zslF = 3 * mu * Fzf / CaF; // tangent of critical slip
    double zslR = 3 * mu * Fzr / CaR; // tangent of critical slip

    // Fiala tire model lateral forces
    double Fyf = std::abs(zF) < zslF ? CaF * zF * (-1 + (std::abs(zF) / zslF) - (std::pow(zF, 2) / 3 * std::pow(zslF, 2))) : -mu * Fzf * sgn(alphaF);
    double Fyr = std::abs(zR) < zslR ? CaR * zR * (-1 + (std::abs(zR) / zslR) - (std::pow(zR, 2) / 3 * std::pow(zslR, 2))) : -mu * Fzr * sgn(alphaR);

    // Absolute velocity, V = Vx/cos(beta)
    double V = input[0] / std::cos(x[2]);

    // Dynamic model (no longitudinal forces since wheel longitudinal stiffness not given in project description)
    dxdt[0] = V * std::cos(x[4] + x[2]);
    dxdt[1] = V * std::sin(x[5] + x[2]);
    dxdt[2] = (/*Fxf * std::sin(input[1] - x[2]) +*/ Fyf * std::cos(x[5] - x[2]) + Fyr * std::cos(x[2]) /*- Fxr * std::sin(x[2])*/ - M * x[3] * V) / (M * V);
    dxdt[3] = (/*a * Fxf * std::sin(input[1]) +*/ a * Fyf * std::cos(x[5]) - b * Fyr) / Iz;
    dxdt[4] = x[3];
    dxdt[5] = input[1]; // input[1] is rate of change of steering angle
}

/**
 * @brief TODO
 *
 * @tparam getInputs
 * @param x state vector (x y theta phi)
 * @param dxdt time derivative of state vector
 */
template <std::vector<double> (*getInput)(void)> void bicycleRearDriveFrontSteerKinematic(const std::vector<double> &x, std::vector<double> &dxdt, const double t) {
    std::vector<double> input = (*getInput)();
    dxdt[0]                   = input[0] * std::cos(x[2]);
    dxdt[1]                   = input[0] * std::sin(x[2]);
    dxdt[2]                   = input[0] * std::tan(x[3]) / L;
    dxdt[3]                   = input[1];
}

} // namespace Models

#endif // MODELS_HPP