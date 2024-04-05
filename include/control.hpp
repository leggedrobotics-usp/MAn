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

// Just to include control refenreces used in this code
#include <control/references.hpp>

// Implementation of the controllers
#include <control/variables.hpp>
#include <control/utils.hpp>
#include <control/simple.hpp>
#include <control/advanced.hpp>

namespace control
{

    void init(mjModel *m, mjData *d)
    {
        jnt_names = mj::joint_names(m, d);
    }

    void prepare_controller_selector()
    {
        // PREPARATION SECTION

        double time = 0;
        double duration = 10;

        // // After last time + duration seconds of simulation time, use model_based_feedforward_control_arm2
        // time += duration;
        // time_barrier.push_back(time);
        // ctrl_names.push_back("Model-based Feedforward");
        // ctrl_functions.push_back(model_based_feedforward_control_arm2);
        // active_control.push_back(false);

        // // After last time + duration seconds of simulation time, use interaction_force_feedback_control_arm2
        // time += duration;
        // time_barrier.push_back(time);
        // ctrl_names.push_back("Interaction Force Feedback");
        // ctrl_functions.push_back(interaction_force_feedback_control_arm2);
        // active_control.push_back(false);

        // After last time + duration seconds of simulation time, use acceleration_feedback_control_arm2
        // time += duration;
        // time_barrier.push_back(time);
        // ctrl_names.push_back("Acceleration Feedback");
        // ctrl_functions.push_back(acceleration_feedback_control_arm2);
        // active_control.push_back(false);

        // Setting up graphics
        time_qpos = basic::figures["time_qpos"];
        time_torques = basic::figures["time_ffwd"];
        time_interaction_force = basic::figures["time_fi"];

        // Initialize accumulated energy average
        mju_fill(control::accumulated_energy_avg, 0, 2);
    }

    void clear_control_variables()
    {
        if (control::qMr)
            delete control::qMr;
        if (control::DqM)
            delete control::DqM;
        if (control::pred_d)
            mj_deleteData(control::pred_d); // Delete prediction data

        control::qMr = nullptr;
        control::DqM = nullptr;
        control::pred_d = nullptr;
    }

    /// @brief Controller selector for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void controller_selector_arm2(const mjModel *m, mjData *d)
    {

        logger::append("time", d->time);
        // logger::append(jnt_names, d->qpos);

        // SECTION 0
        // HUMAN REFERENCE ROUTINES

        // Human ARM (2 DoF)
        // simple_sin_position_arm2(m, d); // just position
        new_position_arm2(m, d);

        // Calculating Interaction Force
        interaction_force_arm2(m, d, control::fi, 2, 2);

        // SECTION I
        // ARM ROBOT CONTROLLER ROUTINES
        // Robot ARM (2 DoF)

        // Check if we have proper sizes
        assert(time_barrier.size() == ctrl_functions.size() && ctrl_functions.size() == active_control.size());

        // Run proper control
        int selected_ctrlid = -1;
        for (int ctrlid = 0; ctrlid < ctrl_functions.size(); ++ctrlid)
        {
            if (d->time >= time_barrier[ctrlid])
            {
                selected_ctrlid = ctrlid;
            }
        }

        // If any controller is seleted, then
        if (selected_ctrlid > -1)
        {
            if (active_control[selected_ctrlid] == false)
            {
                printf("Activating control %s\n", ctrl_names[selected_ctrlid].c_str());
                set_controller_name(ctrl_names[selected_ctrlid]);
                active_control[selected_ctrlid] = true;
            }
            ctrl_functions[selected_ctrlid](m, d);

            // Apply Desired Actuator Torques as Effective Torques
            mju_copy(control::Tr, control::DTr, 2);

            // Apply Actuator Dynamics | TODO
            mju_copy(d->ctrl + 2, control::Tr, 2);
        }

        // SECTION II

        // Calculating average energy
        for (int i = 0; i < 2; ++i)
        {
            if (control::energy_b[i].size < control::energy_b[i].max_size)
            {
                double val = control::fi[i] * control::fi[i];
                energy_avg_acc[i] += val; // O(1)
                // double val = 0;
                // printf("Before bufferd_push_back\n");
                bufferd_push_back(&control::energy_b[i], val); // O(1)
            }
            else
            {
                double val = control::fi[i] * control::fi[i];
                energy_avg_acc[i] += val;                                                           // O(1)
                bufferd_push_and_pop(&control::energy_b[i], val, &control::energy_last_element[i]); // O(1)
                energy_avg_acc[i] -= energy_last_element[i];                                        // O(1)
            }
            // printf("add %d == %d\n", input_vector[i], avg_acc);
            energy_avg_qnt[i] = static_cast<double>(energy_b[i].size);
        }
        // mju_addTo(control::energy_avg_acc, control::fi, 2);
        mju::mju_div(control::energy_avg_val, control::energy_avg_acc, control::energy_avg_qnt, 2);

        // SECTION III
        // PLOTTING ROUTINES

        // Position
        time_qpos->append("human_qpos_shoulder", d->time, d->qpos[0]);
        time_qpos->append("human_qpos_elbow", d->time, d->qpos[1]);
        time_qpos->append("robot_qpos_shoulder", d->time, d->qpos[2]);
        time_qpos->append("robot_qpos_elbow", d->time, d->qpos[3]);

        // Interaction Force
        mju_copy(variables_to_plot, fi, 2);

        // Applied torques
        mju_copy(variables_to_plot + 2, d->qfrc_actuator, 2); // Human Torques
        // mju_copy(variables_to_plot + 2, d->qfrc_applied, 2); // Human Torques
        mju_copy(variables_to_plot + 4, d->qfrc_actuator + 4, 2); // Robot Torques

        // time_torques->append(variables_to_plot_names[2], d->time, d->qfrc_actuator[0]);
        // time_torques->append(variables_to_plot_names[3], d->time, d->qfrc_actuator[1]);
        // time_torques->append(variables_to_plot_names[4], d->time, d->qfrc_actuator[2]);
        // time_torques->append(variables_to_plot_names[5], d->time, d->qfrc_actuator[3]);

        // Interaction Energy
        mju_copy(variables_to_plot + 6, control::energy_avg_val, 2);
        // logger::append("human_interaction_energy_avg_shoulder", control::energy_avg_val[0]);
        // logger::append("human_interaction_energy_avg_elbow", control::energy_avg_val[1]);
        // logger::append("avg_total_human_interaction_energy", control::energy_avg_val[0] + control::energy_avg_val[1]);
        mju_addTo(control::accumulated_energy_avg, control::energy_avg_val, 2);
        logger::append("accumulated_energy_avg", std::log10(control::accumulated_energy_avg[0] + control::accumulated_energy_avg[1]));
        // time_interaction_force->append("human_interaction_energy_avg_shoulder", d->time, variables_to_plot[6]);
        // time_interaction_force->append("human_interaction_energy_avg_elbow", d->time, variables_to_plot[7]);
        time_interaction_force->append("human_interaction_force_0", d->time, control::fi[0]);
        time_interaction_force->append("human_interaction_force_1", d->time, control::fi[1]);

        // logger::append("human_interaction_force_0", control::fi[0]);
        // logger::append("human_interaction_force_1", control::fi[1]);

        time_torques->append("Act_2", d->time, d->qfrc_actuator[2]);
        // time_torques->append("Act_3", d->time, d->qfrc_actuator[3]);
    }

}

// install control callback | THIS usage was causing segmentation fault with the figure implementation
// mjfGeneric mjcb_control = control::simple_sin_torques_arm2;
// mjfGeneric mjcb_control = control::controller_selector_arm2;
// mjfGeneric mjcb_control = nullptr; // No control

#endif // __CONTROL__H_
