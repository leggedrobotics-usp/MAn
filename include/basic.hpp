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
#include <condition_variable>
#include <cassert>
#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <csv/csv.hpp>
#ifdef USE_OPENCV
#include <opencv4/opencv2/opencv.hpp>
#endif

#include <graphics.hpp>
#include <graphics/time_qpos.hpp>
#include <graphics/time_interaction_force.hpp>
#include <graphics/time_feedforward_force.hpp>

bool Exit = false;
const bool high_quality_encoding = false;
const bool high_quality_rendering = false;
bool save_to_csv = true;
bool show_fps = true;
bool show_plot_figure = true;
bool video_record = false;
csv::csv_writer *writer = nullptr;
std::vector<graphics::Figure*> figures;
double end_time = 20.0; // Change this value
// double end_time = std::numeric_limits<double>::max(); // Use this for undefined time

#endif // __BASIC__H_
