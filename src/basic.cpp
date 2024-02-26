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

// Includes for basic simulation
#include <basic.hpp>
#include <graphics.hpp>
#include <graphics/time_qpos.hpp>
#include <graphics/time_interaction_force.hpp>
#include <graphics/time_feedforward_force.hpp>

std::mutex mu, muvideo;
std::mutex video_frame_mtx;
std::condition_variable condv, condvideo;
bool ready = false;
bool Exit = false;
const bool high_quality_encoding = false;
const bool high_quality_rendering = false;
bool save_to_csv = true;
bool show_plot_figure = true;
bool video_record = false;
csv::csv_writer *writer = nullptr;
graphics::FigureTimeQpos *time_qpos = nullptr;
graphics::FigureTimeInteractionForce *time_interaction_force = nullptr;
graphics::FigureTimeFeedForwardForce *time_feedforward_force = nullptr;

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

#ifdef USE_OPENCV
cv::Size video_size(RENDER_WIDTH, RENDER_HEIGHT);
cv::Mat video_frame;
cv::VideoWriter video;
#endif

void runSimulation(mjModel *model, mjData *data)
{
    while (!Exit)
    {
        {
            std::unique_lock<std::mutex> lock(mu);
            condv.wait(lock, []
                       { return ready; });
            if (Exit)
                break;
            // Simulation step
            mj_step(model, data);
            if (save_to_csv && writer)
            {
                writer->append(data->time);
                writer->append(data->qpos, model->nq);
            }
            if (show_plot_figure)
            {
                time_qpos->update();
                time_interaction_force->update();
                time_feedforward_force->update();
            }
            ready = false;
        }
        condv.notify_one();
    }
}

void render(mjModel *model, mjData *data)
{
    // init GLFW, create window, make OpenGL context current, request v-sync
    glfwInit();
    GLFWwindow *window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "MAn: Motion Antecipation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);

    // initialize visualization data structures
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;

    mjv_defaultFreeCamera(model, &cam);
    mjv_defaultPerturb(&pert);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    // cam.trackbodyid = 0;
    // cam.type = mjCAMERA_TRACKING;

    // create scene and context using copied model
    mjv_makeScene(model, &scn, 1000);
    mjr_makeContext(model, &con, mjFONTSCALE_100);

    mjr_setBuffer(mjFB_OFFSCREEN, &con);
    mjr_resizeOffscreen(RENDER_WIDTH, RENDER_HEIGHT, &con);

    mjrRect render_viewport = {0, 0, RENDER_WIDTH, RENDER_HEIGHT};
    mjrRect viewport = {0, 0, 0, 0};
    mjrRect recording_label_viewport = {0, RENDER_HEIGHT - 30, 120, 30};
    mjrRect figure_viewport0 = {0, RENDER_HEIGHT - RENDER_HEIGHT / 2, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
    mjrRect figure_viewport1 = {RENDER_WIDTH - RENDER_WIDTH / 2, RENDER_HEIGHT - RENDER_HEIGHT / 2, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
    mjrRect figure_viewport2 = {0, 0, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};
    mjrRect figure_viewport3 = {RENDER_WIDTH - RENDER_WIDTH / 2, 0, RENDER_WIDTH / 2, RENDER_HEIGHT / 2};

    // run main rendering loop
    while (!glfwWindowShouldClose(window))
    {
        {
            std::unique_lock<std::mutex> lock(mu);
            condv.wait(lock, []
                       { return !ready; });
            // Update simulation data in the copied model and data
            mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);

            // Get GLFW framebuffer viewport size
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
            // Fix recording_label_viewport position
            recording_label_viewport.bottom = viewport.height - 30;

            // render the scene
            mjr_setBuffer(mjFB_OFFSCREEN, &con);
            mjr_render(figure_viewport0, &scn, &con);
            mjr_figure(figure_viewport1, time_qpos->get(), &con);
            mjr_figure(figure_viewport2, time_interaction_force->get(), &con);
            mjr_figure(figure_viewport3, time_feedforward_force->get(), &con);
            mjr_blitBuffer(render_viewport, viewport, 1, 0, &con);

            ready = true;
        }
        condv.notify_one();

        if (video_record)
        {
            // Get rendered OpenGL frame to video frame
            video_frame_mtx.lock();

#ifdef USE_OPENCV
            mjr_readPixels(video_frame.data, nullptr, render_viewport, &con);
#endif

            video_frame_mtx.unlock();

            condvideo.notify_one();

            // render recording label to screen
            mjr_setBuffer(mjFB_WINDOW, &con);
            mjr_label(recording_label_viewport, mjFONT_NORMAL, "Recording...", 1, 1, 0, 1, 1, 0, 0, &con);
        }

        // swap OpenGL buffers
        glfwSwapBuffers(window);

        // process GUI events and callbacks
        glfwPollEvents();
    }
    mjr_restoreBuffer(&con);
    Exit = true;
    condvideo.notify_one();
    {
        std::unique_lock<std::mutex> lock(mu);
        ready = true;
    }
    condv.notify_one();

    // cleanup GLFW and visualization structures
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    exit(0);
}

void video_thread(mjModel *m, mjData *d)
{
    if (!video_record)
        return;

    video_frame = cv::Mat::zeros(RENDER_HEIGHT, RENDER_WIDTH, CV_8UC3);

    // Set video info the same as the simulation model
#ifdef USE_OPENCV
    int fourcc_ = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
    if (high_quality_encoding)
    {
        fourcc_ = cv::VideoWriter::fourcc('h', 'e', 'v', '1');
    }
    // video.set(cv::CAP_PROP_HW_ACCELERATION, cv::VIDEO_ACCELERATION_ANY);
    std::vector<int> params;
    // params.push_back(cv::CAP_PROP_HW_ACCELERATION);
    // params.push_back(cv::VIDEO_ACCELERATION_ANY);
    video.open("video_out.mp4", cv::CAP_FFMPEG, fourcc_, 1.0 / m->opt.timestep, video_size, params);
#endif

    while (!Exit)
    {
        // Wait for new frame to be available from OpenGL
        std::unique_lock<std::mutex> lock(muvideo);
        condv.wait(lock);
        if (Exit)
            break;

        video_frame_mtx.lock();

#ifdef USE_OPENCV
        // Convert data | This should be better done in GPU... Maybe using FFMPEG/libavfilter
        cv::flip(video_frame, video_frame, 0);
        cv::cvtColor(video_frame, video_frame, cv::COLOR_RGB2BGR);

        // Save it to video file | This step could take advantage from GPU Encoding...  Maybe using FFMPEG/libavutil*
        video.write(video_frame);
#endif
        video_frame_mtx.unlock();
    }

#ifdef USE_OPENCV
    video.release();
#endif
}

int main()
{
    // preparing controller selector
    control::prepare_controller_selector();

    // Load original model and data
    mjModel *m = mj_loadXML("model/arm2.xml", NULL, NULL, 0);
    mjData *d = mj_makeData(m);
    m->vis.quality.offsamples = high_quality_rendering ? 4 : 1;

    time_qpos = new graphics::FigureTimeQpos(m, d);
    time_interaction_force = new graphics::FigureTimeInteractionForce(d, control::variables_to_plot, 0,
                                                                      2, control::variables_to_plot_names);
    time_feedforward_force = new graphics::FigureTimeFeedForwardForce(d, control::variables_to_plot, 2,
                                                                      2, control::variables_to_plot_names);

    // Saving log to CSV file
    writer = new csv::csv_writer("log_arm2.csv");
    std::vector<std::string> jnt_names = mj::joint_names(m, d);

    std::vector<std::string> headers;
    headers.push_back("time");

    headers.insert(headers.end(), jnt_names.begin(), jnt_names.end());

    // printf("nq %d njnt %d headers %ld\n", m->nq, m->njnt, headers.size());
    writer->set_headers(headers);

    // Start simulation and rendering threads
    std::thread simThread(runSimulation, m, d);
    std::thread renderThread(render, m, d);
    std::thread videoThread(video_thread, m, d);

    // Join threads
    if (simThread.joinable())
        simThread.join();
    if (renderThread.joinable())
        renderThread.join();
    if (videoThread.joinable())
        videoThread.join();

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);

    delete time_qpos;
    delete time_interaction_force;
    delete time_feedforward_force;

    return 0;
}
