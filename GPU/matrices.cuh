#pragma once

#include <iostream>
#include <cuda.h>

struct Mat
{
    Mat() = delete;
    Mat(int height, int width);
    Mat(int height, int width, float value);
    Mat(float* list_init, int, int);
    Mat(float* list_init, int);
    Mat(const Mat& src);

    ~Mat();

    static Mat eye(int dim);

    Mat dot(const Mat& other);
    Mat cpu_dot(const Mat& other);
    Mat gpu_dot(const Mat& other);
    Mat T();
    Mat cpu_T();
    Mat gpu_T();

    void operator=(const Mat& other);
    Mat operator+(const Mat& other) const;
    Mat cpu_plus(const Mat& other) const;
    Mat gpu_plus(const Mat& other) const;
    Mat operator-(const Mat& other) const;
    Mat cpu_minus(const Mat& other) const;
    Mat gpu_minus(const Mat& other) const;

    Mat inverse() const;

    void print() const;
    Mat copy() const;

    int m_height;
    int m_width;

    float* m_buffer;
    static std::size_t maxThreadsPerBlock;
};
