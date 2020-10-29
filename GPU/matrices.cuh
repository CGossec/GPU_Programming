#pragma once

#include <iostream>
#include <cuda.h>
#include <Eigen/Eigenvalues>

struct EigenVal{
    float val;
    float* vect;
};

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
    // Mat mean() const;

    // __global__ void operator+(const Mat&, Mat*) const;
    // __global__ void operator-(const Mat&, Mat*) const;
    // __global__ void operator*(const float&, Mat*) const;
    // __global__ void operator*(const Mat&, Mat*) const;

    // EigenVal* eigen() const;
    // Mat inverse() const;

    void print() const;
    // __global__ void copy() const;

    int m_height;
    int m_width;

    float* m_buffer;
};
