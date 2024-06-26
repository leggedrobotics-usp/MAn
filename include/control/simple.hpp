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

#ifndef __CONTROL_SIMPLE__H_
#define __CONTROL_SIMPLE__H_

#include <control/variables.hpp>
#include <control/utils.hpp>

namespace control
{
    /***********************************************************************************
     *                                SIMPLE CONTROLLERS                               *
     ***********************************************************************************/

    /// @brief Simple sin position controller for arbitrary number of DoF == actuators
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void simple_sin_position(const mjModel *m, mjData *d)
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

    /// @brief Simple sin position controller for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void simple_sin_position_arm2(const mjModel *m, mjData *d)
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

    /// @brief Sine phase position controller for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void sin_phase_position_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // Your control law may come here like this one
            d->ctrl[0] = M_PI_2 * sin(t * M_PI * f); // Just position
            d->ctrl[1] = M_PI_2 * sin(t * M_PI * f + M_PI_2); // Just position

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

    /// @brief New sin position controller for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void new_sin_position_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 1 second
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        const mjtNum initial_shoulder_angle = 0;
        const mjtNum initial_shoulder_angle_rad = initial_shoulder_angle * M_PI / 180.0;

        const mjtNum initial_elbow_angle = 70;
        const mjtNum initial_elbow_angle_rad = initial_elbow_angle * M_PI / 180.0;

        const mjtNum shoulder_amplitude = 50;
        const mjtNum shoulder_amplitude_rad = shoulder_amplitude * M_PI / 180.0;

        const mjtNum elbow_amplitude = 50;
        const mjtNum elbow_amplitude_rad = elbow_amplitude * M_PI / 180.0;
        const mjtNum step_offset = 0;
        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // if (t < step_offset)
            // {
            //     d->ctrl[0] = initial_shoulder_angle_rad;
            //     d->ctrl[1] = initial_elbow_angle_rad;
            // }
            // else
            if (t >= step_offset)
            {
                // Your control law may come here like this one
                d->ctrl[0] = initial_shoulder_angle_rad + shoulder_amplitude_rad * sin((t - step_offset) * M_PI * f);                  // Just position
                d->ctrl[1] = initial_elbow_angle_rad + elbow_amplitude_rad * sin((t - step_offset) * M_PI * f); // Just position
            }

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

    /// @brief Simple sin position controller for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void new_position_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 2; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // Your control law may come here like this one
            if (t > 5.0)
            {
                d->ctrl[1] = M_PI_2;
                // mju_fill(d->ctrl, M_PI * sin(t * M_PI * f), 2); // Just position
            }

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

}

#endif // __CONTROL_SIMPLE__H_