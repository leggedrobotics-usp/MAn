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

#include <control/variables.hpp>
#include <control/utils.hpp>
#include <control/simple.hpp>
#include <control/advanced.hpp>

namespace control
{

    void prepare_controller_selector()
    {
        // PREPARATION SECTION

        double time = 0;
        double duration = 2;

        // After last time + duration seconds of simulation time, use model_based_feedforward_control_arm2
        time += duration;
        time_barrier.push_back(time);
        ctrl_names.push_back("model_based_feedforward_control_arm2");
        ctrl_functions.push_back(model_based_feedforward_control_arm2);
        active_control.push_back(false);

        // After last time + duration seconds of simulation time, use model_based_feedforward_control_arm2
        time += duration;
        time_barrier.push_back(time);
        ctrl_names.push_back("interaction_force_feedback_control_arm2");
        ctrl_functions.push_back(interaction_force_feedback_control_arm2);
        active_control.push_back(false);
    }

    /// @brief Controller selector for arm2.xml model
    /// @param m - MuJoCo model pointer
    /// @param d - MuJoCo data pointer
    void controller_selector_arm2(const mjModel *m, mjData *d)
    {

        // SECTION 0
        // HUMAN REFERENCE ROUTINES

        // Human ARM (2 DoF)
        simple_sin_position_arm2(m, d); // just position

        // Interaction Force
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
                active_control[selected_ctrlid] = true;
            }
            ctrl_functions[selected_ctrlid](m, d);
        }

        // SECTION II
        // PLOTTING ROUTINES

        // Interaction Force
        mju_copy(variables_to_plot, fi, 2);

        // Applied torques
        mju_copy(variables_to_plot + 2, d->ctrl + 4, 2);
    }

}

// install control callback
// mjfGeneric mjcb_control = control::simple_sin_torques_arm2;
mjfGeneric mjcb_control = control::controller_selector_arm2;
// mjfGeneric mjcb_control = nullptr; // No control

#endif // __CONTROL__H_
