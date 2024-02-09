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

#ifndef __CONTROL__H_
#define __CONTROL__H_

#include <string>
#include <vector>
#include <mujoco/mujoco.h>
#define MAX_CONTROL_VARIABLES_TO_PLOT 12
#define CONTROL_VARIABLES_TO_PLOT 4
namespace control
{
    mjtNum variables_to_plot[MAX_CONTROL_VARIABLES_TO_PLOT];
    int n_variables_to_plot = CONTROL_VARIABLES_TO_PLOT;
    std::vector<std::string> variables_to_plot_names = {"interaction_force_shoulder", "interaction_force_elbow", "robot_torque_shoulder", "robot_torque_elbow"};

    /// @brief Simple sin torque generation for arbitrary number of DoF == actuators
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void simple_sin_torques(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        // Applying stiffness for position control oscilating
        if (m->nu == m->nq)
        {
            // Your control law may come here like this one
            mju_scl(d->ctrl, d->qpos, M_PI * sin(t * M_PI / f), m->nu);
        }
    }

    /// @brief Simple sin torque generation for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void simple_sin_torques_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // Your control law may come here like this one
            mju_fill(d->ctrl, M_PI * sin(t * M_PI * f), 2); // Just position

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

    /// @brief Interaction control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void interaction_control_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {

            // Human ARM (2 DoF)
            mju_fill(d->ctrl, M_PI * sin(t * M_PI * f), 2); // Just position

            // Robot ARM (2 DoF)

            // Calculating Interaction Force
            mjtNum fi[2];
            mjtNum dp[2]; // (da-db) * k
            mjtNum dv[2]; // (dva-dvb) * b

            // solving for stiffness
            mju_sub(dp, d->qpos + 2, d->qpos, 2);       // dp = (da-db)
            mju_scl(dp, dp, m->tendon_stiffness[0], 2); // dp = (da-db) * k

            // solving for damping
            mju_sub(dv, d->qpos + 2, d->qpos, 2);     // dv = (dva-dvb)
            mju_scl(dv, dv, m->tendon_damping[0], 2); // dv = (dva-dvb) * b

            // fi = dp + dv
            // fi = (da-db) * k + (dva-dvb) * b
            mju_add(fi, dp, dv, 2); // fi = dp + dv
            mju_copy(variables_to_plot, fi, 2);

            if (d->time < 2.0)
            {
                // do nothing in Robot ARM
                mju_fill(d->ctrl + 4, 0.0, 2); // zero torques
            }
            else
            {
                // control offset d->ctrl+4 | ppvv|mm
                // feedforward
                mju_fill(d->ctrl + 4, 0.0, 2); // zero torques
                // robot_arm_torques = human_acc * robot_mass
                // mju_scl(d->ctrl + 4, d->qacc, 0.5, 2);
            }

            // With interaction force offset
            // mju_copy(variables_to_plot + 2, d->ctrl + 4, 2);
            // mju_fill(variables_to_plot, 0, 2);
            // mju_fill(variables_to_plot + 2, 10, 2);

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

}

// install control callback
// mjfGeneric mjcb_control = control::simple_sin_torques_arm2;
mjfGeneric mjcb_control = control::interaction_control_arm2;

#endif // __CONTROL__H_
