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

#ifndef __LOGGER__H_
#define __LOGGER__H_

#include <csv/csv.hpp>
#include <basic.hpp>

namespace logger
{
    csv::csv_writer *writer = nullptr;

    void init(std::string file_name, mjModel *model, mjData *data)
    {
        writer = new csv::csv_writer(file_name);
        std::vector<std::string> jnt_names = mj::joint_names(model, data);

        std::vector<std::string> headers;
        headers.push_back("time");

        headers.insert(headers.end(), jnt_names.begin(), jnt_names.end());

        // printf("nq %d njnt %d headers %ld\n", m->nq, m->njnt, headers.size());
        writer->set_headers(headers);
        printf("INITIALIZED LOGGER\n");
    }

    void step(mjModel *model, mjData *data)
    {
        if (save_to_csv && writer)
        {
            writer->append(data->time);
            writer->append(data->qpos, model->nq);
        }
    }

    void finish(mjModel *m, mjData *d)
    {
        delete writer;
        writer = nullptr;
        printf("FINISHED LOGGER\n");
    }
}

#endif // __LOGGER__H_
