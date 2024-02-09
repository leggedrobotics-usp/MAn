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

#ifndef __GRAPHICS_TIME_INTERACTION_FORCE__H_
#define __GRAPHICS_TIME_INTERACTION_FORCE__H_

#include <string>
#include <cstring>
#include <vector>
#include <mujoco/mujoco.h>

namespace graphics
{
    class FigureTimeInteractionForce : public Figure
    {
    public:
        mjData *d;
        mjtNum *var_to_plot;
        int nvar;
        int offset;

        FigureTimeInteractionForce(mjData *d, mjtNum *var_to_plot, int offset,
                                   int nvar, std::vector<std::string> &var_legends) : d(d), var_to_plot(var_to_plot),
                                                                                      offset(offset), nvar(nvar)
        {
            init();
            set_title("Time x Interaction Forces");
            set_xlabel("Time (s)");

            // Setting legends for interaction forces
            std::vector<std::string> legends;
            legends.assign(var_legends.begin() + offset, var_legends.begin() + offset + nvar);
            set_legends(legends);
        }

        void update()
        {
            append(d->time, var_to_plot + offset);
        }
    };
}

#endif // __GRAPHICS_TIME_INTERACTION_FORCE__H_
