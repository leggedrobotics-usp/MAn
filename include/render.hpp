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

#ifndef __RENDER__H_
#define __RENDER__H_

#include <string>
#include <cstring>
#include <vector>
#include <chrono>
#include <mujoco/mujoco.h>
#include <video.hpp>
#include <basic.hpp>
#include <graphics.hpp>

// Set controller name label width
#define CONTROLLER_NAME_WIDTH 400

// Set initial window size
#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720

// SD
// #define RENDER_WIDTH 640
// #define RENDER_HEIGHT 480

// HD
// #define RENDER_WIDTH 1280
// #define RENDER_HEIGHT 720

// FHD
#define RENDER_WIDTH 1920
#define RENDER_HEIGHT 1080

// UHD 4K
// #define RENDER_WIDTH 1920*2
// #define RENDER_HEIGHT 1080*2

namespace render
{
    typedef struct mjRender_s
    {
        mjvCamera cam;
        mjvOption opt;
        mjvScene scn;
        mjrContext con;
        mjvPerturb pert;
        mjrRect view;
    } mjRender;

    mjRender r;

    mjrRect render_viewport;
    mjrRect viewport;
    mjrRect controller_name_label_viewport;
    mjrRect recording_label_viewport;
    mjrRect fps_label_viewport;
    mjrRect figure_viewport0;
    mjrRect figure_viewport1;
    mjrRect figure_viewport2;
    mjrRect figure_viewport3;

    std::chrono::steady_clock::time_point last_time, current_time, initial_time = std::chrono::steady_clock::now();

    double last_rendered_sim_time = -1; // d->time from previous step
    std::chrono::steady_clock::time_point last_rendered_time;

    int last_fps = 0;

    GLFWwindow *window = nullptr;

    void mj_initRender(mjRender *r, mjModel *m)
    {
        // initialize render data structures
        mjv_defaultFreeCamera(m, &r->cam);
        mjv_defaultPerturb(&r->pert);
        mjv_defaultOption(&r->opt);
        mjr_defaultContext(&r->con);

        // create scene and context using copied model
        mjv_makeScene(m, &r->scn, 1000);
        mjr_makeContext(m, &r->con, mjFONTSCALE_100);
    }

    void mj_resizeRender(mjRender *r, mjrRect *v)
    {
        memcpy(&r->view, v, static_cast<size_t>(sizeof(mjrRect)));

        // set render to framebuffer and resize it
        mjr_setBuffer(mjFB_OFFSCREEN, &r->con);
        mjr_resizeOffscreen(r->view.width, r->view.height, &r->con);
    }

    void mj_freeRender(mjRender *r)
    {
        mjv_freeScene(&r->scn);
        mjr_freeContext(&r->con);
    }

    bool should_render_frame(mjModel *m, mjData *d)
    {
        bool should = false;

        // Render frames accordingly with video fps
        if (video_record)
        {
            if (should_write_frame(m, d))
                should = true;
        }
        else
        {
            // Render real-time checking with simulation
            if (real_time)
            {
                {
                    std::chrono::duration<double> diff = current_time - initial_time;

                    if (d->time >= diff.count())
                    {
                        should = true;
                    }
                }

                // Render real-time fps checking
                {
                    std::chrono::duration<double> diff = current_time - last_rendered_time;

                    if (diff.count() >= target_render_time)
                    {
                        should = should && true;
                        last_rendered_time = current_time;
                    }
                }
            }
            else
            {
                // Render real-time fps checking
                {
                    std::chrono::duration<double> diff = current_time - last_rendered_time;

                    if (diff.count() >= target_render_time)
                    {
                        should = true;
                        last_rendered_time = current_time;
                    }
                }

                // Render simulation checking
                {
                    double diff = d->time - last_rendered_sim_time;

                    if (diff >= target_render_time)
                    {
                        should = true;
                        last_rendered_sim_time = d->time;
                    }
                }
            }
        }
        return should;
    }

    void init(mjModel *model, mjData *data)
    {
        if (!show_window)
            return;

        // init GLFW, create window, make OpenGL context current, request v-sync
        glfwInit();
        window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "MAn: Motion Antecipation", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
        // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);

        // initialize visualization data structures
        mj_initRender(&r, model);

        mjr_setBuffer(mjFB_OFFSCREEN, &r.con);
        render_viewport = {0, 0, RENDER_WIDTH, RENDER_HEIGHT};
        mj_resizeRender(&r, &render_viewport);

        viewport = {0, 0, 0, 0};
        controller_name_label_viewport = {RENDER_WIDTH - RENDER_WIDTH / 2 - CONTROLLER_NAME_WIDTH / 2, RENDER_HEIGHT - 30, CONTROLLER_NAME_WIDTH, 30};
        recording_label_viewport = {0, WINDOW_HEIGHT - 30, 120, 30};
        fps_label_viewport = {0, 0, 80, 30};
        figure_viewport0 = {0, RENDER_HEIGHT - RENDER_HEIGHT / 2, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
        figure_viewport1 = {RENDER_WIDTH - RENDER_WIDTH / 2, RENDER_HEIGHT - RENDER_HEIGHT / 2, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
        figure_viewport2 = {0, 0, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
        figure_viewport3 = {RENDER_WIDTH - RENDER_WIDTH / 2, 0, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
        current_time = std::chrono::steady_clock::now();
    }

    void render_fps(mjData *d)
    {
        // Get time difference
        std::chrono::duration<double> diff = current_time - last_time;
        last_time = current_time;

        // Get compensated FPS value
        int instant_fps = static_cast<int>(1.0 / diff.count());
        int compensated_fps = static_cast<int>(last_fps * 0.9 + instant_fps * 0.1);
        last_fps = compensated_fps;

        // Get Real-Time Factor
        std::chrono::duration<double> cur_real_time = current_time - initial_time;
        double rtf = d->time / cur_real_time.count();

        // Prepare text to render
        std::stringstream ss;

        // Frames Per Second and Real-Time Factor
        ss << "fps: " << compensated_fps << " (rtf: " << std::setprecision(3) << rtf << ")";

        // Render FPS text on window buffer
        mjr_setBuffer(mjFB_WINDOW, &r.con);
        mjr_label(fps_label_viewport, mjFONT_NORMAL, ss.str().c_str(), 0, 0, 0, 0, 1, 1, 1, &r.con);
    }

    void step(mjModel *model, mjData *data)
    {
        if (!show_window)
            return;
        if (glfwWindowShouldClose(window))
        {
            Exit = true;
            return;
        }
        // Set current time
        current_time = std::chrono::steady_clock::now();

        // run main rendering
        if (should_render_frame(model, data))
        {
            // Update simulation data in the copied model and data
            mjv_updateScene(model, data, &r.opt, NULL, &r.cam, mjCAT_ALL, &r.scn);

            // Get GLFW framebuffer viewport size
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            // Fix recording_label_viewport position
            recording_label_viewport.bottom = viewport.height - 30;

            // render the scene
            mjr_setBuffer(mjFB_OFFSCREEN, &r.con);
            mjr_render(figure_viewport0, &r.scn, &r.con);

            if (show_plot_figure)
            {
                for (graphics::Figure *f : graphics::figures_to_render)
                {
                    f->update();
                }
                if (graphics::figures_to_render[0])
                    mjr_figure(figure_viewport1, graphics::figures_to_render[0]->get(), &r.con);
                if (graphics::figures_to_render[1])
                    mjr_figure(figure_viewport2, graphics::figures_to_render[1]->get(), &r.con);
                if (graphics::figures_to_render[2])
                    mjr_figure(figure_viewport3, graphics::figures_to_render[2]->get(), &r.con);
            }

            if (show_controller_name)
            {
                mjr_label(controller_name_label_viewport, mjFONT_NORMAL, controller_name.c_str(), 1, 1, 1, 0.5, 0, 0, 0, &r.con);
            }

            mjr_blitBuffer(r.view, viewport, 1, 0, &r.con);

            if (video_record)
            {
                // Get rendered OpenGL frame to video frame

                if (should_write_frame(model, data))
                {
#ifdef USE_OPENCV
                    mjr_readPixels(video::video_frame.data, nullptr, r.view, &r.con);
#endif
                }

                // render recording label to screen
                mjr_setBuffer(mjFB_WINDOW, &r.con);
                mjr_label(recording_label_viewport, mjFONT_NORMAL, "Recording...", 1, 1, 0, 1, 1, 0, 0, &r.con);
            }

            if (show_fps)
            {
                render_fps(data);
            }

            // swap OpenGL buffers
            glfwSwapBuffers(window);
        }

        // process GUI events and callbacks
        glfwPollEvents();
    }

    void finish()
    {
        if (!show_window)
            return;
        // cleanup GLFW and visualization structures
        glfwTerminate();
        mjv_freeScene(&r.scn);
        mjr_freeContext(&r.con);
        printf("FINISHED RENDER\n");
    }
}

#endif // __RENDER__H_
