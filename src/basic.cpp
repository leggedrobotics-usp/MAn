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
#include <simulation.hpp>
#include <render.hpp>

int main()
{
    // set_render_fps(120); // 120 Hz for exaple, if timestep is ~8.33ms
    // set_render_fps(60); // 60 Hz for exaple, if timestep is ~16,66ms
    set_render_fps(30); // 30 Hz for exaple, if timestep is ~33.33ms

    // preparing controller selector
    control::prepare_controller_selector();

    // Load original model and data
    mjModel *m = mj_loadXML("model/arm2.xml", NULL, NULL, 0);
    mjData *d = mj_makeData(m);
    m->vis.quality.offsamples = high_quality_rendering ? 4 : 1;

    figures.push_back(new graphics::FigureTimeQpos(m, d));
    figures.push_back(new graphics::FigureTimeInteractionForce(d, control::variables_to_plot, 0,
                                                               2, control::variables_to_plot_names));
    figures.push_back(new graphics::FigureTimeFeedForwardForce(d, control::variables_to_plot, 2,
                                                               2, control::variables_to_plot_names));

    // Saving log to CSV file
    // writer = new csv::csv_writer("log_arm2.csv");
    // std::vector<std::string> jnt_names = mj::joint_names(m, d);

    // std::vector<std::string> headers;
    // headers.push_back("time");

    // headers.insert(headers.end(), jnt_names.begin(), jnt_names.end());

    // printf("nq %d njnt %d headers %ld\n", m->nq, m->njnt, headers.size());
    // writer->set_headers(headers);

    render::init(m, d);
    video::init(m, RENDER_WIDTH, RENDER_HEIGHT);

    while (!Exit && d->time <= end_time)
    {
        simulation::step(m, d);
        render::step(m, d);
        video::step(m, d);
    }

    video::finish();
    render::finish();
    simulation::finish(m, d);

    return 0;
}
