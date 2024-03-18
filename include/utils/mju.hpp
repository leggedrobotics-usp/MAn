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

#ifndef __UTILS_MJU__H_
#define __UTILS_MJU__H_

#include <cassert>
#include <mujoco/mujoco.h>
#define mjUSEAVX

#ifdef mjUSEAVX
#include <immintrin.h>
#endif

namespace mju
{
  // res = vec1 .* vec2 : Element-wise multiplication
  void mju_mul(mjtNum *res, const mjtNum *vec1, const mjtNum *vec2, int n)
  {
    int i = 0;

#ifdef mjUSEAVX
    int n_4 = n - 4;

    // vector part
    if (n_4 >= 0)
    {
      __m256d mul, val1, val2;

      // parallel computation
      while (i <= n_4)
      {
        val1 = _mm256_loadu_pd(vec1 + i);
        val2 = _mm256_loadu_pd(vec2 + i);
        mul = _mm256_mul_pd(val1, val2);
        _mm256_storeu_pd(res + i, mul);
        i += 4;
      }
    }

    // process remaining
    int n_i = n - i;
    if (n_i == 3)
    {
      res[i] = vec1[i] * vec2[i];
      res[i + 1] = vec1[i + 1] * vec2[i + 1];
      res[i + 2] = vec1[i + 2] * vec2[i + 2];
    }
    else if (n_i == 2)
    {
      res[i] = vec1[i] * vec2[i];
      res[i + 1] = vec1[i + 1] * vec2[i + 1];
    }
    else if (n_i == 1)
    {
      res[i] = vec1[i] * vec2[i];
    }

#else
    for (; i < n; i++)
    {
      res[i] = vec1[i] * vec2[i];
    }
#endif
  }

  // res = vec1 ./ vec2 : Element-wise division
  void mju_div(mjtNum *res, const mjtNum *vec1, const mjtNum *vec2, int n)
  {
    int i = 0;

#ifdef mjUSEAVX
    int n_4 = n - 4;

    // vector part
    if (n_4 >= 0)
    {
      __m256d div, val1, val2;

      // parallel computation
      while (i <= n_4)
      {
        val1 = _mm256_loadu_pd(vec1 + i);
        val2 = _mm256_loadu_pd(vec2 + i);
        div = _mm256_div_pd(val1, val2);
        _mm256_storeu_pd(res + i, div);
        i += 4;
      }
    }

    // process remaining
    int n_i = n - i;
    if (n_i == 3)
    {
      res[i] = vec1[i] / vec2[i];
      res[i + 1] = vec1[i + 1] / vec2[i + 1];
      res[i + 2] = vec1[i + 2] / vec2[i + 2];
    }
    else if (n_i == 2)
    {
      res[i] = vec1[i] / vec2[i];
      res[i + 1] = vec1[i + 1] / vec2[i + 1];
    }
    else if (n_i == 1)
    {
      res[i] = vec1[i] / vec2[i];
    }

#else
    for (; i < n; i++)
    {
      res[i] = vec1[i] / vec2[i];
    }
#endif
  }
}

#endif // __UTILS_MJU__H_
