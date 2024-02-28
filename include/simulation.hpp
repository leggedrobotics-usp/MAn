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

#ifndef __SIMULATION__H_
#define __SIMULATION__H_

#include <basic.hpp>

namespace simulation
{
    void init()
    {
    }

    void step(mjModel *model, mjData *data)
    {
        // Simulation step
        mj_step(model, data);
        // if (save_to_csv && writer)
        // {
        //     writer->append(data->time);
        //     writer->append(data->qpos, model->nq);
        // }
        if (show_plot_figure)
        {
            for (graphics::Figure *f : figures)
            {
                if (f)
                    f->update();
            }
        }
    }

    void finish(mjModel *m, mjData *d)
    {
        // Cleanup
        mj_deleteData(d);
        mj_deleteModel(m);
        printf("FINISHED SIMULATION\n");
    }
}

#endif // __SIMULATION__H_
