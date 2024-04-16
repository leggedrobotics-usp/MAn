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
     *                          ADVANCED CONTROLLERS FUNCTIONS                         *
     ***********************************************************************************/

    /// @brief Model-based feedforward control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void model_based_feedforward_control_arm2(const mjModel *m, mjData *d)
    {
        mjtNum time_tau = mju_max(m->actuator_dynprm[mjNDYN * 2], m->actuator_dynprm[mjNDYN * 3]);
        int nsteps = static_cast<int>((2 * time_tau) / m->opt.timestep);

        // Calculate feedforward (Tff) | frff from eq. 5 in ref. [1] using inertia instead of mass
        feedforward_arm2(m, d, control::Tff, 2, 2, use_prediction, nsteps);

        // Set control with feedforward
        mju_copy(control::DTr, control::Tff, 2); // DTr = Tff
    }

    /// @brief Interaction force feedback control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void interaction_force_feedback_control_arm2(const mjModel *m, mjData *d)
    {

        mjtNum tmp[2];
        mjtNum kp[2] = {100, 100};
        mjtNum kd[2] = {10, 10};

        // Use tendon values
        // mju_copy(kp, m->tendon_stiffness, 2);
        // mju_copy(kd, m->tendon_damping, 2);

        // Calculate feedforward (Tff) | frff from eq. 5 in ref. [1] using inertia instead of mass
        feedforward_arm2(m, d, control::Tff, 2, 2);

        // Set DTr = Tff
        mju_copy(control::DTr, control::Tff, 2);

        // Calculate fi
        interaction_force_arm2(m, d, control::fi, 2, 2);

        // Calculate dfi
        derivative_interaction_force_arm2(m, d, control::dfi, 2, 2);

        // Calculate DTr = Tff + (kp * fi)
        mju_copy(control::kp, kp, 2);
        mju::mju_mul(tmp, control::kp, control::fi, 2);
        mju_add(control::DTr, control::DTr, tmp, 2);

        // Calculate DTr = Tff + (kp * fi) + (kd * dfi)
        mju_copy(control::kd, kd, 2);
        mju::mju_mul(tmp, control::kd, control::fi, 2);
        mju_add(control::DTr, control::DTr, tmp, 2);
    }

    /// @brief Acceleration Feedback Control for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void acceleration_feedback_control_arm2(const mjModel *m, mjData *d)
    {

        mjtNum tmp[2];
        mjtNum kp[2] = {0.0001, 0.0001};
        mjtNum ki[2] = {0.0000126, 0.0000126};
        mjtNum time_tau = mju_max(m->actuator_dynprm[mjNDYN * 2], m->actuator_dynprm[mjNDYN * 3]);
        int nsteps = static_cast<int>((1 * time_tau) / m->opt.timestep);
        // printf("nsteps = %d\n", nsteps);

        // Calculate Feedforward Torques (Tff) | frff from eq. 5 in ref. [1] using inertia instead of mass
        feedforward_arm2(m, d, control::Tff, 2, 2, use_prediction, nsteps);

        // Set Tfb = 0
        mju_fill(control::Tfb, 0, 2); // Tfb = 0

        // Calculate Tfb = kp * (ddq_h - ddq_r)
        mju_copy(control::kp, kp, 2);
        mju_sub(tmp, d->qacc, d->qacc + 2, 2);  // tmp = (ddq_h - ddq_r)
        mju::mju_mul(tmp, control::kp, tmp, 2); // tmp = kp * (ddq_h - ddq_r)
        mju_addTo(control::Tfb, tmp, 2);        // Tfb = kp * (ddq_h - ddq_r)

        // Calculate Tfb = kp * (ddq_h - ddq_r) + ki * (dq_h - dq_r)
        mju_copy(control::ki, ki, 2);
        mju_sub(tmp, d->qvel, d->qvel + 2, 2);  // tmp = (dq_h - dq_r)
        mju::mju_mul(tmp, control::ki, tmp, 2); // tmp = ki * (dq_h - dq_r)
        mju_addTo(control::Tfb, tmp, 2);        // Tfb = kp * (ddq_h - ddq_r) + ki * (dq_h - dq_r)

        // time_torques->append("Tfb_0", d->time, Tfb[0]);
        // time_torques->append("Tfb_1", d->time, Tfb[1]);

        // Set control with DTr = Tff + Tfb
        mju_add(control::DTr, control::Tff, control::Tfb, 2);
        // if (d->time > 5.0)
        //     control::DTr[1] = 1;

        time_torques->append("DTr_0", d->time, DTr[0]);
        // time_torques->append("DTr_1", d->time, DTr[1]);
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