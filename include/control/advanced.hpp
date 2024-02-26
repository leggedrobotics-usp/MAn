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

#ifndef __CONTROL_ADVANCED__H_
#define __CONTROL_ADVANCED__H_

#include <control/variables.hpp>
#include <control/utils.hpp>

namespace control
{
    /***********************************************************************************
     *                          ADVANCED CONTROLLERS FUNCTIONS                          *
     ***********************************************************************************/

    /// @brief Model-based feedforward control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void model_based_feedforward_control_arm2(const mjModel *m, mjData *d)
    {
        feedforward(m, d, d->ctrl + 4, 2, 2);
    }

    /// @brief Interaction force feedback control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void interaction_force_feedback_control_arm2(const mjModel *m, mjData *d)
    {
        mjtNum mass[2] = {0.3, 0.1};

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {

            // Robot ARM (2 DoF)

            // Calculate dfi - we assume fi is already being calculated
            mju_sub(dfi, control::fi, control::last_fi, 2);                // dfi = fi - last_fi
            mju_scl(dfi, dfi, 1.0 / m->opt.timestep, 2); // dfi = (fi - last_fi)/dt

            if (d->time < 2.0)
            {
                // do nothing in Robot ARM
                mju_fill(d->ctrl + 4, 0.0, 2); // zero torques
            }
            else
            {
                // control offset d->ctrl+4 | ppvv|mm
                // feedforward
                // robot_arm_torques = human_acc * robot_mass
                // mju_scl(d->ctrl + 4, d->qacc, 0.5, 2);
            }

            // Copy current force interaction to last force interaction
            // To be used in force interaction
            mju_copy(control::last_fi, control::fi, 2);

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);
        }
    }

    /// @brief Advanced Interaction control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void advanced_interaction_control_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 4; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        static mjtNum Tau = 0;
        static mjtNum Kp = 100;
        static mjtNum Kv = 10;

        static mjtNum Tau2 = 0;
        static mjtNum Kp2 = 10;
        static mjtNum Kv2 = 1;

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // Calculating Interaction Force
            mjtNum fi[2];
            mjtNum dp[2]; // (da-db) * k
            mjtNum dv[2]; // (dva-dvb) * b

            // solving for stiffness
            mju_sub(dp, d->qpos, d->qpos + 2, 2);         // dp = (da-db)
            mju::mju_mul(dp, dp, m->tendon_stiffness, 2); // dp = (da-db) * k

            // solving for damping
            mju_sub(dv, d->qpos, d->qpos + 2, 2);       // dv = (dva-dvb)
            mju::mju_mul(dv, dv, m->tendon_damping, 2); // dv = (dva-dvb) * b

            // fi = dp + dv
            // fi = (da-db) * k + (dva-dvb) * b
            mju_add(fi, dp, dv, 2); // fi = dp + dv
            mju_copy(variables_to_plot, fi, 2);

            d->ctrl[0] = M_PI * sin(t * M_PI * 0.5f * f);
            d->ctrl[1] = M_PI * sin(t * M_PI * 1.0f * f);

            Tau = Kp * (d->qpos[0] - d->qpos[0 + 2]) + Kv * (d->qvel[0] - d->qvel[0 + 2]);
            Tau2 = Kp2 * (d->qpos[1] - d->qpos[1 + 2]) + Kv2 * (d->qvel[1] - d->qvel[1 + 2]);

            d->qfrc_applied[0 + 2] = Tau - 0.3 * d->qacc[0];  // Tau  - 0.25*d->qacc[0]; Jacobiano desde Mujoco?
            d->qfrc_applied[1 + 2] = Tau2 - 0.1 * d->qacc[1]; // Tau2 - 0.1*d->qacc[1];

            // printf("T: %.5f T2: %.5f \n",Tau,Tau2);

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);

            // Plotting copy
            // Interaction Force
            mju_copy(variables_to_plot, fi, 2);

            // Applied Force
            mju_copy(variables_to_plot + 2, d->qfrc_applied + 2, 2);
        }
    }

    /// @brief Advanced Predictive Interaction Control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void advanced_predictive_interaction_control_arm2(const mjModel *m, mjData *d)
    {
        const mjtNum period = 4; // 2 seconds
        const mjtNum f = 1.0 / period;
        mjtNum t = d->time;

        static mjtNum Tau[2] = {0};
        static mjtNum Kp[2] = {100, 10}; // Kp
        static mjtNum Kv[2] = {10, 1};   // Ki
        static mjtNum mass[2] = {0.3, 0.1};

        const static double time_barrier[2] = {10, 20};
        static bool active_control[3] = {0};

        // Applying stiffness for position control oscilating
        if (m->nq == 4)
        {
            // Calculating Interaction Force
            mjtNum fi[2];
            mjtNum dp[2]; // (da-db) * k
            mjtNum dv[2]; // (dva-dvb) * b

            // solving for stiffness
            mju_sub(dp, d->qpos, d->qpos + 2, 2);         // dp = (da-db)
            mju::mju_mul(dp, dp, m->tendon_stiffness, 2); // dp = (da-db) * k

            // solving for damping
            mju_sub(dv, d->qpos, d->qpos + 2, 2);       // dv = (dva-dvb)
            mju::mju_mul(dv, dv, m->tendon_damping, 2); // dv = (dva-dvb) * b

            // fi = dp + dv
            // fi = (da-db) * k + (dva-dvb) * b
            mju_add(fi, dp, dv, 2); // fi = dp + dv
            mju_copy(variables_to_plot, fi, 2);

            d->ctrl[0] = M_PI * sin(t * M_PI * 1.0f * f);
            d->ctrl[1] = M_PI * sin(t * M_PI * 1.0f * f);

            // Tau = Kp * (d->qpos[0] - d->qpos[0 + 2]) + Kv * (d->qvel[0] - d->qvel[0 + 2]);
            // Tau2 = Kp2 * (d->qpos[1] - d->qpos[1 + 2]) + Kv2 * (d->qvel[1] - d->qvel[1 + 2]);
            mju_copy(d->qfrc_applied + 2, Tau, 2);

            mjtNum dap[2]; // (daa-dab) * kp
            mjtNum dvi[2]; // (dva-dvb) * ki

            if (d->time < time_barrier[0])
            { // No interaction control
                if (!active_control[0])
                {
                    printf("No interaction control\n");
                    active_control[0] = true;
                }
            }
            else if (d->time >= time_barrier[0] && d->time < time_barrier[1])
            { // delayed acceleration
                if (!active_control[1])
                {
                    printf("delayed acceleration\n");
                    active_control[1] = true;
                }
                mju_copy(d->qfrc_applied + 2, Tau, 2);
                mjtNum ff[2]; // mass * acc
                // mju::mju_mul(ff, mass, acc, 2);

                // d->qfrc_applied[0 + 2] = Tau - 3.6 * d->qacc[0];  // Tau  - 0.25*d->qacc[0]; Jacobiano desde Mujoco?
                // d->qfrc_applied[1 + 2] = Tau2 - 3.6 * d->qacc[1]; // Tau2 - 0.1*d->qacc[1];

                // mju_add(d->qfrc_applied, d->qfrc_applied, ff);
                // mju_sub(d->qfrc_applied, d->qfrc_applied, ff);
            }
            else if (d->time >= time_barrier[1])
            { // predicted acceleration
                if (!active_control[2])
                {
                    printf("predicted acceleration\n");
                    active_control[2] = true;
                }
                /************************************************************************
                 *                    Use MuJoCo for predictive control                  *
                 *************************************************************************/
                mjfGeneric previous_mjcb_control = mjcb_control; // Save control pointer
                mjcb_control = nullptr;                          // No control

                // Not very efficient, but should be effective
                mjData *d_pred = mj_copyData(nullptr, m, d); // Copy simulation states.

                // Using only step2 because step1 was already called for 'd'
                mj_forward(m, d_pred);

                // double dt = m->opt.timestep;
                // m->opt.timestep = dt*0.5;
                mj_step(m, d_pred); // Predict acceleration in next step

                // m->opt.timestep = dt; // Recover original timestep
                mjcb_control = previous_mjcb_control; // Recover control pointer

                /************************************************************************
                 *            Use MuJoCo calculated acceleration as prediction           *
                 *************************************************************************/
                // d->qfrc_applied[0 + 2] = Tau - 3.6 * d_pred->qacc[0];  // Tau  - 0.25*d_pred->qacc[0]; Jacobiano desde Mujoco?
                // d->qfrc_applied[1 + 2] = Tau2 - 3.6 * d_pred->qacc[1]; // Tau2 - 0.1*d_pred->qacc[1];

                mj_deleteData(d_pred); // Delete prediction data
            }
            // printf("T: %.5f T2: %.5f \n",Tau,Tau2);

            // Print control vector for debugging
            // mju_printMat(d->ctrl, 1, m->nu);

            // Plotting copy
            // Interaction Force
            mju_copy(variables_to_plot, fi, 2);

            // Applied Feedforward Force
            mju_copy(variables_to_plot + 2, d->qfrc_applied + 2, 2);
        }
    }

}

#endif // __CONTROL_ADVANCED__H_