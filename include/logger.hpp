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
    std::vector<std::string> headers;
    std::map<std::string, double> header_values;
    const double NaN = std::nan("0"); // NaN - Not-a-Number constant

    void init(std::string file_name)
    {
        writer = new csv::csv_writer(file_name);
        printf("INITIALIZED LOGGER\n");
    }

    inline void init_headers()
    {
        if (writer->cols.size() == 0)
            writer->set_headers(headers);
    }

    void step()
    {
        if (save_to_csv && writer)
        {
            // Init headers if not initialized
            init_headers();

            // For all headers, get current values
            for (std::string &header : headers)
            {
                writer->append(header_values[header]);
                header_values[header] = NaN;
            }
        }
    }

    void finish(mjModel *m, mjData *d)
    {
        delete writer;
        writer = nullptr;
        printf("FINISHED LOGGER\n");
    }

    inline void append(std::string name, double value)
    {
        if (header_values.find(name) == header_values.end())
            headers.push_back(name);
        header_values[name] = value;
    }

    void append(std::vector<std::string> &names, double *values)
    {
        for (size_t i = 0; i < names.size(); ++i)
        {
            append(names[i], values[i]);
        }
    }
}

#endif // __LOGGER__H_
