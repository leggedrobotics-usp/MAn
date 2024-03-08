// Circular Buffer (https://github.com/lomcin/avg_test)

// MIT License

// Copyright (c) 2023 Lucas Oliveira Maggi

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#pragma once
#include <stdlib.h>
#include <assert.h>

// Only use this if you know what you are doing!!!
// #define NO_ASSERT


/******************************************************************************************
 *                                                                                        *
 *                                  INT32 VALUES                                          *
 *                                                                                        *
 ******************************************************************************************/
typedef struct bufferi_st
{
    int *data;       // buffer data pointer
    size_t max_size; // maximum circular buffer size
    size_t size;     // circular buffer size
    size_t cur;      // cursor position
} bufferi_t;

// If you want a memory deallocation look for bufferi_free(bufferi_t *b).
// This is just a quick clear, without writing zeros, the data is still available.
void bufferi_clear(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // initializing size as 0
    b->size = 0;

    // initializing cursor as 0
    b->cur = 0;
}

void bufferi_init(bufferi_t *b, size_t max_size)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // allocate the requested size
    b->data = (int *)malloc(max_size * sizeof(int));

    // check if data allocation was successful
    if (b->data != NULL)
    {
        // setting up a valid max_size after checking allocation
        b->max_size = max_size;
    }
    else
    {
        // setting up a valid max_size after failing allocation
        b->max_size = 0;
    }

    // initializing size and cur as 0
    bufferi_clear(b);
}

void bufferi_free(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if data is allocated before trying to deallocate
    assert(b->data != NULL);
#endif

    // free data
    free(b->data);

    // just making sure the previous pointer is invalid
    b->data = NULL;

    // making sure the max_size verifications will be coherent
    b->max_size = 0;

    // making sure the size and cur verifications will be coherent
    bufferi_clear(b);
}

int *bufferi_at(bufferi_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);
#endif

    size_t circular_pos = (pos + b->cur) % b->max_size;

    // check if position is valid in buffer data
    // assert(circular_pos > -1 && circular_pos < b->max_size);

    // return pointer to circular position in buffer data
    return &(b->data[circular_pos]);
}

int bufferi_get(bufferi_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);

    // check if position is valid
    assert((pos >= 0) && (pos < b->size));
#endif

    // return value from circular position in buffer data
    return *bufferi_at(b, pos);
}

void bufferi_push_back(bufferi_t *b, int value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not full
    assert(b->size < b->max_size);
#endif

    // get possible position for new value
    int *data = bufferi_at(b, b->size);

    // set data with value
    *data = value;

    // increment the circular buffer size
    b->size++;
}

void bufferi_pop_front(bufferi_t *b, int *value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not empty
    assert(b->size > 0);
#endif

    // get possible position for first value
    int *data = bufferi_at(b, 0);

    // sets value from first element in the circular buffer
    if (value != NULL)
    {
        *value = *data;
    }

    // decrement the circular buffer size
    b->size--;

    // move buffer cursor
    b->cur = (b->cur + 1) % b->max_size;
}

void bufferi_print(bufferi_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    printf("bufferi:");
    for (size_t p = 0; p < b->size; ++p)
    {
        printf(" %d", bufferi_get(b, p));
    }
    printf("\n");
}

void bufferi_push_and_pop(bufferi_t *b, int push_value, int *pop_value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // pop front before overflowing
    bufferi_pop_front(b, pop_value);

    // push back after removing first element
    bufferi_push_back(b, push_value);
}

void bufferi_avgi(bufferi_t *b, int *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    int acc = 0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += bufferi_get(b, p);
    }
    *avg = acc / b->size;
}

void bufferi_avgd(bufferi_t *b, double *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    double acc = 0.0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (double)bufferi_get(b, p);
    }
    *avg = acc / ((double)b->size);
}

void bufferi_avgf(bufferi_t *b, float *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    float acc = 0.0f;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (float)bufferi_get(b, p);
    }
    *avg = acc / ((float)b->size);
}


/******************************************************************************************
 *                                                                                        *
 *                                  DOUBLE VALUES                                         *
 *                                                                                        *
 ******************************************************************************************/

typedef struct bufferd_st
{
    double *data;    // buffer data pointer
    size_t max_size; // maximum circular buffer size
    size_t size;     // circular buffer size
    size_t cur;      // cursor position
} bufferd_t;

// If you want a memory deallocation look for bufferd_free(bufferd_t *b).
// This is just a quick clear, without writing zeros, the data is still available.
void bufferd_clear(bufferd_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // initializing size as 0
    b->size = 0;

    // initializing cursor as 0
    b->cur = 0;
}

void bufferd_init(bufferd_t *b, size_t max_size)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // allocate the requested size
    b->data = (double *)malloc(max_size * sizeof(double));

    // check if data allocation was successful
    if (b->data != NULL)
    {
        // setting up a valid max_size after checking allocation
        b->max_size = max_size;
    }
    else
    {
        // setting up a valid max_size after failing allocation
        b->max_size = 0;
    }

    // initializing size and cur as 0
    bufferd_clear(b);
}

void bufferd_free(bufferd_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if data is allocated before trying to deallocate
    assert(b->data != NULL);
#endif

    // free data
    free(b->data);

    // just making sure the previous pointer is invalid
    b->data = NULL;

    // making sure the max_size verifications will be coherent
    b->max_size = 0;

    // making sure the size and cur verifications will be coherent
    bufferd_clear(b);
}

double *bufferd_at(bufferd_t *b, size_t pos)
{

#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);
#endif

    size_t circular_pos = (pos + b->cur) % b->max_size;

    // check if position is valid in buffer data
    // assert(circular_pos > -1 && circular_pos < b->max_size);

    // return pointer to circular position in buffer data
    return b->data + circular_pos;
}

double bufferd_get(bufferd_t *b, size_t pos)
{

#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);

    // check if position is valid
    assert((pos >= 0) && (pos < b->size));
#endif

    // return value from circular position in buffer data
    return *bufferd_at(b, pos);
}

void bufferd_push_back(bufferd_t *b, double value)
{

#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not full
    assert(b->size < b->max_size);
#endif

    // get possible position for new value
    double *data = bufferd_at(b, b->size);

    // set data with value
    *data = value;

    // increment the circular buffer size
    b->size++;
}

void bufferd_pop_front(bufferd_t *b, double *value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not empty
    assert(b->size > 0);
#endif

    // get possible position for first value
    double *data = bufferd_at(b, 0);

    // sets value from first element in the circular buffer
    if (value != NULL)
    {
        *value = *data;
    }

    // decrement the circular buffer size
    b->size--;

    // move buffer cursor
    // printf("b->cur %lud b->max_size %lud\n", b->cur, b->max_size);
    b->cur = (b->cur + 1) % b->max_size;
}

void bufferd_print(bufferd_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    printf("bufferd:");
    for (size_t p = 0; p < b->size; ++p)
    {
        printf(" %lf", bufferd_get(b, p));
    }
    printf("\n");
}

void bufferd_push_and_pop(bufferd_t *b, double push_value, double *pop_value)
{

#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // pop front before overflowing
    bufferd_pop_front(b, pop_value);

    // push back after removing first element
    bufferd_push_back(b, push_value);
}

void bufferd_avgi(bufferd_t *b, int *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    int acc = 0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (int)bufferd_get(b, p);
    }
    *avg = acc / b->size;
}

void bufferd_avgd(bufferd_t *b, double *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    double acc = 0.0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (double)bufferd_get(b, p);
    }
    *avg = acc / ((double)b->size);
}

void bufferd_avgf(bufferd_t *b, float *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    float acc = 0.0f;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (float)bufferd_get(b, p);
    }
    *avg = acc / ((float)b->size);
}


/******************************************************************************************
 *                                                                                        *
 *                                  FLOAT  VALUES                                         *
 *                                                                                        *
 ******************************************************************************************/

typedef struct bufferf_st
{
    float *data;       // buffer data pointer
    size_t max_size; // maximum circular buffer size
    size_t size;     // circular buffer size
    size_t cur;      // cursor position
} bufferf_t;

// If you want a memory deallocation look for bufferf_free(bufferf_t *b).
// This is just a quick clear, without writing zeros, the data is still available.
void bufferf_clear(bufferf_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // initializing size as 0
    b->size = 0;

    // initializing cursor as 0
    b->cur = 0;
}

void bufferf_init(bufferf_t *b, size_t max_size)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // allocate the requested size
    b->data = (float *)malloc(max_size * sizeof(float));

    // check if data allocation was successful
    if (b->data != NULL)
    {
        // setting up a valid max_size after checking allocation
        b->max_size = max_size;
    }
    else
    {
        // setting up a valid max_size after failing allocation
        b->max_size = 0;
    }

    // initializing size and cur as 0
    bufferf_clear(b);
}

void bufferf_free(bufferf_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if data is allocated before trying to deallocate
    assert(b->data != NULL);
#endif

    // free data
    free(b->data);

    // just making sure the previous pointer is invalid
    b->data = NULL;

    // making sure the max_size verifications will be coherent
    b->max_size = 0;

    // making sure the size and cur verifications will be coherent
    bufferf_clear(b);
}

float *bufferf_at(bufferf_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);
#endif

    size_t circular_pos = (pos + b->cur) % b->max_size;

    // check if position is valid in buffer data
    // assert(circular_pos > -1 && circular_pos < b->max_size);

    // return pointer to circular position in buffer data
    return &(b->data[circular_pos]);
}

float bufferf_get(bufferf_t *b, size_t pos)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if buffer has data
    assert(b->data != NULL);

    // check if position is valid
    assert((pos >= 0) && (pos < b->size));
#endif

    // return value from circular position in buffer data
    return *bufferf_at(b, pos);
}

void bufferf_push_back(bufferf_t *b, float value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not full
    assert(b->size < b->max_size);
#endif

    // get possible position for new value
    float *data = bufferf_at(b, b->size);

    // set data with value
    *data = value;

    // increment the circular buffer size
    b->size++;
}

void bufferf_pop_front(bufferf_t *b, float *value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if circular buffer is not empty
    assert(b->size > 0);
#endif

    // get possible position for first value
    float *data = bufferf_at(b, 0);

    // sets value from first element in the circular buffer
    if (value != NULL)
    {
        *value = *data;
    }

    // decrement the circular buffer size
    b->size--;

    // move buffer cursor
    b->cur = (b->cur + 1) % b->max_size;
}

void bufferf_print(bufferf_t *b)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    printf("bufferf:");
    for (size_t p = 0; p < b->size; ++p)
    {
        printf(" %f", bufferf_get(b, p));
    }
    printf("\n");
}

void bufferf_push_and_pop(bufferf_t *b, float push_value, float *pop_value)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);
#endif

    // pop front before overflowing
    bufferf_pop_front(b, pop_value);

    // push back after removing first element
    bufferf_push_back(b, push_value);
}

void bufferf_avgi(bufferf_t *b, int *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    int acc = 0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (int) bufferf_get(b, p);
    }
    *avg = acc / b->size;
}

void bufferf_avgd(bufferf_t *b, double *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    double acc = 0.0;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (double)bufferf_get(b, p);
    }
    *avg = acc / ((double)b->size);
}

void bufferf_avgf(bufferf_t *b, float *avg)
{
#ifndef NO_ASSERT
    // check if buffer is not null
    assert(b != NULL);

    // check if avg return is not null
    assert(avg != NULL);
#endif

    float acc = 0.0f;
    for (size_t p = 0; p < b->size; ++p)
    {
        acc += (float)bufferf_get(b, p);
    }
    *avg = acc / ((float)b->size);
}