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

#ifndef __BASIC__H_
#define __BASIC__H_

#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>
#include <cassert>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#ifdef USE_OPENCV
#include <opencv4/opencv2/opencv.hpp>
#endif

std::string controller_name = "No controller";
namespace control
{

    void set_controller_name(std::string &ctrl_name)
    {
        controller_name = ctrl_name;
    }

}
#include <graphics.hpp>
#include <graphics/time_generic.hpp>

bool Exit = false;
const bool high_quality_encoding = false;
const bool high_quality_rendering = false;
const bool use_simulation_fps_for_video = false;
bool save_to_csv = true;
bool real_time = false;
bool show_window = true;
bool show_fps = true;
bool show_plot_figure = true;
bool show_controller_name = true;
bool video_record = false;
long long int video_frames_written = 0;
double target_render_fps = 60.0;
double target_render_time = 1.0 / target_render_fps;
long long int sim_step = -1;

double end_time = 24.0; // Change this value
// double end_time = std::numeric_limits<double>::max(); // Use this for undefined time

void set_render_fps(double fps)
{
    target_render_fps = fps;
    target_render_time = 1.0 / fps;
}

bool should_write_frame(mjModel *m, mjData *d)
{
    return (video_frames_written == static_cast<long long int>(sim_step * target_render_fps * m->opt.timestep));
}

namespace basic
{
    std::map<std::string, graphics::Figure *> figures;
}
#endif // __BASIC__H_
