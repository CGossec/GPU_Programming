#pragma once

#include <iostream>
#include <cuda.h>
#include <Eigen/Eigenvalues>

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
    Mat T();

    Mat operator+(const Mat& other) const;
    Mat operator-(const Mat& other) const;

    Mat inverse() const;

    void print() const;
    Mat copy() const;

    int m_height;
    int m_width;

    float* m_buffer;
};
