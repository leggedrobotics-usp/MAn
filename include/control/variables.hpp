/*
MIT License

Copyright (c) 2024 Lucas Maggi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __CONTROL_VARIABLES__H_
#define __CONTROL_VARIABLES__H_

#include <string>
#include <vector>
#include <logger.hpp>
#include <basic.hpp>
#include <utils/circular_buffer.h>

#define MAX_CONTROL_VARIABLES_TO_PLOT 12
#define CONTROL_VARIABLES_TO_PLOT 6
#define WINDOW_SIZE 1000 // 1 s for simulations with dt = 1ms

namespace control
{
    //// Plotting variables
    mjtNum variables_to_plot[MAX_CONTROL_VARIABLES_TO_PLOT];
    const int n_variables_to_plot = CONTROL_VARIABLES_TO_PLOT;
    std::vector<std::string> variables_to_plot_names = {"interaction_force_shoulder", "interaction_force_elbow", "human_torque_shoulder", "human_torque_elbow", "robot_torque_shoulder", "robot_torque_elbow", "interaction_energy_shoulder", "interaction_energy_elbow"};

    //// Controller Selector variables
    std::vector<double> time_barrier;
    std::vector<bool> active_control;
    std::vector<mjfGeneric> ctrl_functions;
    std::vector<std::string> ctrl_names;

    // Simulation
    
    std::vector<std::string> jnt_names;

    //// Shared Controller/Dynamics Variables

    // Forces // Deprecated

    static mjtNum fr[2] = {0};  // force exerted by the robot
    static mjtNum fi[2] = {0};  // current force interaction
    static mjtNum dfi[2] = {0}; // interaction force first derivative

    // Torques

    static mjtNum DTr[2] = {0}; // Desired Torque exerted by the robot
    static mjtNum Tr[2] = {0};  // Torque exerted by the robot
    static mjtNum Ti[2] = {0};  // Current Torque Interaction
    static mjtNum dTi[2] = {0}; // Interaction Torque first derivative
    static mjtNum Tff[2] = {0}; // FeedForward Torque
    static mjtNum Tfb[2] = {0}; // FeedBack Torque

    // Constants

    static mjtNum ka[2] = {0}; // ka | springs coefficients
    static mjtNum kp[2] = {0}; // kp | proportional coefficients
    static mjtNum ki[2] = {0}; // ki | integral coefficients
    static mjtNum kd[2] = {0}; // kd | derivative coefficients

    // Extras

    int nDqM = 0;          // Dense Inertia Matrix Size
    int nqMr = 0;          // Robot Inertia Matrix Size
    mjtNum jacp[12] = {0}; // Jacobian matrix for position
    mjtNum jacv[12] = {0}; // Jacobian matrix for velocity
    mjtNum *DqM = nullptr; // Dense Inertia Matrix
    mjtNum *qMr = nullptr; // Robot Inertia Matrix

    // Prediction

    mjData *pred_d = nullptr;

    // Energy Metrics

    double accumulated_energy_avg[2] = {0};
    double energy_last_element[2] = {0};
    double energy_avg_acc[2] = {0};
    double energy_avg_qnt[2] = {0};
    double energy_avg_val[2] = {0};
    bufferd_t energy_b[2] = {0}; // buffer struct

    // Figures

    graphics::Figure *time_qpos = nullptr;
    graphics::Figure *time_torques = nullptr;
    graphics::Figure *time_interaction_force = nullptr;

}

#endif // __CONTROL_VARIABLES__H_