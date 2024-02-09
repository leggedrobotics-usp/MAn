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

#ifndef __GRAPHICS_FIGURE__H_
#define __GRAPHICS_FIGURE__H_

#include <mujoco/mujoco.h>

namespace graphics
{
    class Figure
    {
    public:
        mjvFigure *f;
        int nlines = 0;
        int npts = 0;
        bool auto_range = true;
        std::thread t;

        Figure() = default;

        ~Figure()
        {
            if (t.joinable())
                t.join();
            delete this->f;
        }

        mjvFigure *get()
        {
            if (t.joinable())
                t.join();
            return this->f;
        }

        void init()
        {
            if (this->f == nullptr)
                this->f = new mjvFigure();
            mjv_defaultFigure(this->f);
            strcpy(this->f->xformat, "%.2lf");
            this->f->flg_ticklabel[0] = 1;
            this->f->flg_ticklabel[1] = 1;
            this->f->figurergba[3] = 1.0;
            this->f->gridsize[0] = 10;
            this->f->gridsize[1] = 10;
        }

        virtual void update() = 0;

        void append(mjtNum xvalue, mjtNum *yvalues)
        {
            t = std::thread([&]()
                            {
                // Calculate the memsize to be moved before appending new values
                size_t memsize = ((npts - 1) * sizeof(mjtNum)) << 1; // << 1 == * 2

                // Move previous points
                if (npts == mjMAXLINEPNT)
                    for (int i = 0; i < nlines; ++i)
                        for (int j = 0; j < ((npts - 1) << 1); ++j)
                            this->f->linedata[i][j] = this->f->linedata[i][j + 2];

                // Update points per line
                npts = std::min(npts + 1, mjMAXLINEPNT);

                // Last (available) position
                int lasti = (npts - 1) << 1; // (npts -1) * 2 : because point has two numbers

                // Append given points to the last position
                for (int i = 0; i < nlines; ++i)
                {
                    // Set each line data appended points
                    this->f->linedata[i][lasti] = xvalue;         // x value
                    this->f->linedata[i][lasti + 1] = yvalues[i]; // y value
                    this->f->linepnt[i] = npts;
                }

                // If using auto ranging, use the first point x value, and the last point x values
                if (auto_range)
                {
                    this->f->range[0][0] = this->f->linedata[0][0];     // min x value
                    this->f->range[0][1] = this->f->linedata[0][lasti]; // max x value
                } });
        }

        void set_title(std::string title)
        {
            strcpy(this->f->title, title.c_str());
        }

        void set_xlabel(std::string xlabel)
        {
            strcpy(this->f->xlabel, xlabel.c_str());
        }

        void set_legends(std::vector<std::string> &legends)
        {
            this->nlines = legends.size();
            assert(this->nlines <= mjMAXLINE);

            const float scale_factor = 1.0 / 255.0;

            for (size_t i = 0; i < this->nlines; ++i)
            {
                // Setting legend name
                strcpy(this->f->linename[i], legends[i].c_str());

                // Clear line
                memset(this->f->linedata[i], 0, sizeof(mjtNum) * mjMAXLINEPNT);

                // Setting legend colors randomly
                float *rgb = this->f->linergb[i];
                rgb[0] = static_cast<float>(rand() % 256) * scale_factor;
                rgb[1] = static_cast<float>(rand() % 256) * scale_factor;
                rgb[2] = static_cast<float>(rand() % 256) * scale_factor;
            }
        }
    };
}

#endif // __GRAPHICS_FIGURE__H_
