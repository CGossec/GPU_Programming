#ifndef GPGPU_ICP_H
#define GPGPU_ICP_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>
#include "matrices.hh"

typedef std::vector<std::tuple<std::size_t, std::size_t>> Correspondences;

class ICP_matlab
{
public:
    ICP_matlab (const Mat& M, const Mat& P);
    ~ICP_matlab();
private:
    Mat get_correspondences(const Mat& P, const Mat& M);
};


#endif //GPGPU_ICP_H
