#pragma once

#include <array>
#include <vector>
#include <iostream>

struct Mat
{
    Mat() = delete;
    Mat(int height, int width);
    Mat(int height, int width, float value);
    Mat(std::vector<std::vector<float>>&& list_init);
    Mat(const Mat& m);

    static Mat eye(int dim);

    Mat dot(const Mat&);
    Mat dot(const std::vector<float>&);
    Mat T();
    Mat mean() const;

    std::vector<float> operator[](const int pos) const;
    std::vector<float>& operator[](const int pos);
    Mat operator+(const Mat&) const;
    Mat operator-(const Mat&) const;
    Mat operator*(const float&) const;
    Mat operator*(const Mat&) const;

    void print() const;
    Mat copy() const;

    int m_height;
    int m_width;
    std::vector<std::vector<float>> m_buffer;
};
