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

    // static Mat eye(int dim);

    // __global__ void dot(const Mat& other, Mat* ret){
    // void dot(const float* other, int height, Mat* ret);
    // __global__ void T(Mat* ret);
    // Mat mean() const;

    // float* operator[](const int pos) const;
    // const float* operator[](const int pos);
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
