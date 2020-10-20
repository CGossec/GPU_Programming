#pragma once

#include <vector>
#include <iostream>

struct Mat
{
    Mat() = delete;
    Mat(int height, int width);
    Mat(int height, int width, float value);

    Mat dot(const Mat&);
    Mat T();

    Mat operator+(const Mat&);
    Mat operator-(const Mat&);
    Mat operator*(const float&);

    void print();

    int m_height;
    int m_width;
    std::vector<std::vector<float>> m_buffer;
};