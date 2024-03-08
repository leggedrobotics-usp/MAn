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
#include <string>
#include <vector>
#include <deque>
// #include <unordered_map>
#include <map>

namespace graphics
{
    class Figure
    {
    public:
        mjvFigure f;
        int nlines = 0;
        bool auto_range = true;
        bool updated_figure = false;
        // std::thread t;

        std::vector<std::string> line_names;
        std::map<std::string, std::deque<double>> line_values;

        ~Figure()
        {
            // if (t.joinable())
            //     t.join();
        }

        mjvFigure *get()
        {
            // if (t.joinable())
            //     t.join();
            return &f;
        }

        void init()
        {
            mjv_defaultFigure(&f);
            strcpy(f.xformat, "%.2lf");
            f.flg_ticklabel[0] = 1;
            f.flg_ticklabel[1] = 1;
            f.figurergba[3] = 1.0;
            f.gridsize[0] = 10;
            f.gridsize[1] = 10;
            line_values.clear();
        }

        void update()
        {
            // t = std::thread([&]()
            // {
            double xmin = std::numeric_limits<double>::infinity();
            double xmax = -std::numeric_limits<double>::infinity();

            // Copy points to figure line data
            for (int l = 0; l < line_names.size(); ++l)
            {
                std::deque<double> &values = line_values[line_names[l]];
                const int npts = values.size() >> 1;
                for (int i = 0; i < npts; ++i)
                {
                    const int ibase = i << 1;
                    const int i2 = ibase + 1;
                    // Set each line data appended points
                    f.linedata[l][ibase] = values[ibase]; // x value
                    f.linedata[l][i2] = values[i2];       // y value
                    f.linepnt[l] = npts;

                    xmin = (values[ibase] < xmin ? values[ibase] : xmin);
                    xmax = (values[ibase] > xmax ? values[ibase] : xmax);
                }
            }

            // If using auto ranging, use the first point x value, and the last point x values
            if (auto_range)
            {
                if (xmin != std::numeric_limits<double>::infinity())
                    f.range[0][0] = xmin; // min x value
                if (xmax != -std::numeric_limits<double>::infinity())
                    f.range[0][1] = xmax; // max x value
            }
            //  });
        }

        void append(std::string name, double xvalue, double yvalue)
        {
            // printf("Looking for '%s'\n", name.c_str());
            auto it = line_values.find(name);
            // printf("%lf %lf\n", xvalue, yvalue);
            // printf("it : (%s, %lf)\n", it->first.c_str(), it->second);
            if (it == line_values.end())
            {
                // printf("Couldn't find '%s', then add a new line\n", name.c_str());
                // // New line
                nlines = line_names.size();
                assert(this->nlines <= mjMAXLINE);
                // Setting legend name
                strcpy(f.linename[nlines], name.c_str());

                // Clear line
                // memset(f.linedata[nlines], 0, sizeof(mjtNum) * (mjMAXLINEPNT << 1));

                // Setting legend colors randomly
                const float scale_factor = 1.0 / 255.0;
                float *rgb = f.linergb[nlines];
                rgb[0] = static_cast<float>((rand() % 32) << 3) * scale_factor;
                rgb[1] = static_cast<float>((rand() % 32) << 3) * scale_factor;
                rgb[2] = static_cast<float>((rand() % 32) << 3) * scale_factor;
                line_names.push_back(name);
                nlines = line_names.size();
            }

            // Already defined line
            line_values[name].push_back(xvalue);
            line_values[name].push_back(yvalue);

            if (line_values[name].size() > (mjMAXLINEPNT << 1))
            {
                line_values[name].pop_front(); // pop first xvalue
                line_values[name].pop_front(); // pop first yvalue
            }
        }

        void set_title(std::string title)
        {
            strcpy(f.title, title.c_str());
        }

        void set_xlabel(std::string xlabel)
        {
            strcpy(f.xlabel, xlabel.c_str());
        }

        void set_legends(std::vector<std::string> &legends)
        {
            //     this->nlines = legends.size();
            //     line_names.assign(legends.begin(), legends.end());
            //     line_values.clear();
            //     for (std::string legend : legends)
            //     {
            //         line_values[legend].clear();
            //     }
            //     assert(this->nlines <= mjMAXLINE);

            //     const float scale_factor = 1.0 / 255.0;
            //     // srand(0);
            //     // srandom(0);

            //     for (size_t i = 0; i < this->nlines; ++i)
            //     {
            //         // Setting legend name
            //         strcpy(f.linename[i], legends[i].c_str());

            //         // Clear line
            //         memset(f.linedata[i], 0, sizeof(mjtNum) * (mjMAXLINEPNT << 1));

            //         // Setting legend colors randomly
            //         float *rgb = f.linergb[i];
            //         rgb[0] = static_cast<float>((rand() % 32) << 3) * scale_factor;
            //         rgb[1] = static_cast<float>((rand() % 32) << 3) * scale_factor;
            //         rgb[2] = static_cast<float>((rand() % 32) << 3) * scale_factor;
            //     }
        }
    };
}

#endif // __GRAPHICS_FIGURE__H_
