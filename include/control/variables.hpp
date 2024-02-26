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

#define MAX_CONTROL_VARIABLES_TO_PLOT 12
#define CONTROL_VARIABLES_TO_PLOT 4

namespace control
{
    // Plotting variables
    mjtNum variables_to_plot[MAX_CONTROL_VARIABLES_TO_PLOT];
    int n_variables_to_plot = CONTROL_VARIABLES_TO_PLOT;
    std::vector<std::string> variables_to_plot_names = {"interaction_force_shoulder", "interaction_force_elbow", "robot_torque_shoulder", "robot_torque_elbow"};

    // Controller Selector variables
    std::vector<double> time_barrier;
    std::vector<bool> active_control;
    std::vector<mjfGeneric> ctrl_functions;
    std::vector<std::string> ctrl_names;

    // Controller variables
    static mjtNum fi[2];             // current force interaction
    static mjtNum last_fi[2];        // last force interaction
    static mjtNum dfi[2];            // interaction force derivative
    static mjtNum kp[2] = {100, 10}; // kp
    static mjtNum kd[2] = {10, 1};   // kd
}

#endif // __CONTROL_VARIABLES__H_