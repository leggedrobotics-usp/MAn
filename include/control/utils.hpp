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
#include <control/simple.hpp>

namespace control
{
    /***********************************************************************************
     *                     ADVANCED CONTROLLERS UTILITY FUNCTIONS                      *
     ***********************************************************************************/

    /// @brief Predict system state for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param nsteps - number of steps
    void predict_arm2(const mjModel *m, mjData *d, int nsteps)
    {
        control::pred_d = mj_copyData(control::pred_d, m, d); // Copy simulation states.
        while (nsteps--)
        {
            // Run first part of step
            mj_step1(m, control::pred_d);

            // Do actuation for this timestep
            control::pred_f(m, control::pred_d);

            // Run second part of step
            mj_step2(m, control::pred_d);
        }
    }

    /// @brief Calculate the Model-based feedforward force for generalized arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param ffwd - feedfordward force (resulting)
    /// @param offset - robot mass offset
    /// @param n - degrees of freedom
    /// @param use_prediction - use predict_arm2 to predict system state in nsteps
    /// @param nsteps - number of steps for prediction
    void feedforward_arm2(const mjModel *m, mjData *d, mjtNum *ffwd, int offset, int n, bool use_prediction = false, int nsteps = 50)
    {

        // Tff = M_r * ddq_h + h(q, dq)

        // Memory management
        {
            // Check DqM memory availability
            if (control::nDqM != m->nv * m->nv)
            {
                if (control::DqM)
                    delete control::DqM;
                control::DqM = new mjtNum[m->nv * m->nv];
                assert(control::DqM);
                control::nDqM = m->nv * m->nv;
            }

            // Check DqM memory availability
            if (control::nqMr != 4)
            {
                if (control::qMr)
                    delete control::qMr;
                control::qMr = new mjtNum[4];
                assert(control::qMr);
                control::nqMr = 4;
            }
        }

        // Feedforward calculation
        {
            mjData *ld = d; // Local Data (for both scenarios, using or not the predicted accelerations)
            if (use_prediction)
            {
                predict_arm2(m, d, nsteps);
                ld = control::pred_d;
            }
            // Get DqM: Dense Inertia Matrix
            mj_fullM(m, control::DqM, d->qM);

            // Set qMr elements from DqM
            control::qMr[0] = control::DqM[8 + 2];
            control::qMr[1] = control::DqM[8 + 3];
            control::qMr[2] = control::DqM[12 + 2];
            control::qMr[3] = control::DqM[12 + 3];

            // time_torques->append("qM_0", ld->time, qMr[0]);
            // time_torques->append("qM_1", ld->time, qMr[1]);
            // time_torques->append("qM_2", ld->time, qMr[2]);
            // time_torques->append("qM_3", ld->time, qMr[3]);

            mju_mulMatVec(ffwd, qMr, ld->qacc, 2, 2);
            mju_addTo(ffwd, d->qfrc_bias + 2, 2);
        }
    }

    /// @brief Calculate the interaction force for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param fi - force of interaction (resulting)
    /// @param offset - robot joints offset
    /// @param n - degrees of freedom
    void interaction_force_arm2(const mjModel *m, mjData *d, mjtNum *fi, int offset, int n)
    {
        // Just copy spring force from MuJoCo
        mju_copy(fi, d->qfrc_spring + 2, 2);
    }

    /// @brief Calculate the first derivative of interaction force for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    /// @param dfi - first derivative for force of interaction (resulting)
    /// @param offset - robot joints offset
    /// @param n - degrees of freedom
    void derivative_interaction_force_arm2(const mjModel *m, mjData *d, mjtNum *dfi, int offset, int n)
    {
        // Calculate dfi using eq. 9 from ref. [1]
        mju_copy(control::ka, m->tendon_stiffness, n); // setting ka = tendon stiffness
        mju_sub(dfi, d->qvel + offset, d->qvel, n);    // temporarily: dfi = dq_r - dq_h
        mju::mju_mul(dfi, dfi, control::ka, n);        // dfi = ka * (dq_r - dq_h)
    }
}

#endif // __CONTROL_UTILS__H_