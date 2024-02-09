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
#include <mujoco/mujoco.h>

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

    void mj_freeRender(mjRender *r) {
        mjv_freeScene(&r->scn);
        mjr_freeContext(&r->con);
    }
}

#endif // __RENDER__H_
