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

#ifndef __CONTROL_UTILS__H_
#define __CONTROL_UTILS__H_

#include <control/variables.hpp>

namespace control
{
    /***********************************************************************************
     *                     ADVANCED CONTROLLERS UTILITY FUNCTIONS                       *
     ***********************************************************************************/

    /// @brief Calculate the Model-based feedforward force for generalized arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param ffwd - feedfordward force (resulting)
    /// @param n - degrees of freedom
    /// @param offset - robot mass offset
    /// @param inertia_type - inertia type: 0 - mass | 1 - inertia
    void feedforward(const mjModel *m, mjData *d, mjtNum *ffwd, int n, int offset, int inertia_type = 0)
    {
        // Please use one or the other:

        if (inertia_type == 0)
        {
            // Mass
            mju::mju_mul(ffwd, m->body_mass + offset, d->qacc, n); // ffwd = m_r * ddq_h
        }
        else if (inertia_type == 1)
        {
            // Inertia
            mju::mju_mul(ffwd, m->dof_M0 + offset, d->qacc, n); // ffwd = M_r * ddq_h
        }
        else
        {
            assert(false); // Invalid inertia type
        }
    }

    /// @brief Calculate the interaction force for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param fi - force of interaction (resulting)
    /// @param n - degrees of freedom
    /// @param offset - robot joints offset
    void interaction_force_arm2(const mjModel *m, mjData *d, mjtNum *fi, int n, int offset)
    {
        mjtNum tmp[n];      // temporary variable
        mju_fill(fi, 0, n); // Filling fi with zeros

        if (kp != nullptr)
        {
            // solving for stiffness
            mju_sub(tmp, d->qpos + offset, d->qpos, n);     // (q_r - q_h)
            mju::mju_mul(tmp, tmp, m->tendon_stiffness, n); // kp * (q_r - q_h)
            mju_add(fi, fi, tmp, n);                        // fi = kp * (q_r - q_h)
        }

        if (kd != nullptr)
        {
            // solving for damping
            mju_sub(tmp, d->qvel + offset, d->qvel, n);   // (dq_r - dq_h)
            mju::mju_mul(tmp, tmp, m->tendon_damping, n); // kd * (dq_r - dq_h)
            mju_add(fi, fi, tmp, n);                      // fi = kp * (q_r - q_h) + kd * (dq_r - dq_h)
        }

        // Full formula
        // fi = kp * (q_r - q_h) + kd * (dq_r - dq_h)
    }
}

#endif // __CONTROL_UTILS__H_