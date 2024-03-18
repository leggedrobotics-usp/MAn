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
#include <mujoco/mujoco.h>
#include <utils/mju.hpp>
#include <utils/circular_buffer.h>

#define MAX_CONTROL_VARIABLES_TO_PLOT 12
#define CONTROL_VARIABLES_TO_PLOT 6
#define WINDOW_SIZE 200 // 1 s for simulations with dt = 5ms

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

    //// Shared Controller Variables

    // Forces
    static mjtNum fr[2] = {0};  // force exerted by the robot
    static mjtNum fi[2] = {0};  // current force interaction
    static mjtNum dfi[2] = {0}; // interaction force first derivative

    // Torques
    static mjtNum tr[2] = {0};   // torque exerted by the robot
    static mjtNum ti[2] = {0};   // current torque interaction
    static mjtNum dti[2] = {0};  // interaction torque first derivative
    static mjtNum ffwd[2] = {0}; // feedforward
    static mjtNum ffb[2] = {0}; // feedback

    // Constants
    static mjtNum ka[2] = {0}; // ka
    static mjtNum kp[2] = {0}; // kp
    static mjtNum kd[2] = {0}; // kd
    static mjtNum ki[2] = {0}; // ki

    // Extras
    mjtNum jacp[12] = {0}; // Jacobian matrix for position
    mjtNum jacv[12] = {0}; // Jacobian matrix for velocity

    // Energy Metrics
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