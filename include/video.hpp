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

#ifndef __VIDEO__H_
#define __VIDEO__H_

#include <basic.hpp>

namespace video
{
#ifdef USE_OPENCV
    cv::Size video_size;
    cv::Mat video_frame;
    cv::VideoWriter video;
#endif

    void init(mjModel *m, int width, int height)
    {
        if (!video_record)
            return;

        video_frame = cv::Mat::zeros(height, width, CV_8UC3);
        video_size = video_frame.size();

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
        // video.open("video_out.mp4", cv::CAP_FFMPEG, fourcc_, 1.0 / m->opt.timestep, video_size, params);
        if (use_simulation_fps_for_video)
            set_render_fps(1.0 / m->opt.timestep); // 200 Hz for exaple, if timestep is 5ms
        video.open("video_out.mp4", cv::CAP_FFMPEG, fourcc_, target_render_fps, video_size);
#endif
    }

    void step(mjModel *m, mjData *d)
    {
        if (!video_record || Exit)
            return;

        if (!should_write_frame(m, d))
            return;
#ifdef USE_OPENCV
        // Convert data | This should be better done in GPU... Maybe using FFMPEG/libavfilter
        cv::flip(video_frame, video_frame, 0);
        cv::cvtColor(video_frame, video_frame, cv::COLOR_RGB2BGR);

        // Save it to video file | This step could take advantage from GPU Encoding...  Maybe using FFMPEG/libavutil*
        video.write(video_frame);
#endif
        video_frames_written++;
    }

    void finish()
    {
        if (!video_record)
            return;
#ifdef USE_OPENCV
        video.release();
#endif
        printf("FINISHED VIDEO\n");
    }
}

#endif // __VIDEO__H_
