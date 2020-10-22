#ifndef GPGPU_ICP_H
#define GPGPU_ICP_H

#include <array>
#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>
#include "matrices.hh"

typedef std::vector<std::tuple<std::size_t, std::size_t>> Correspondences;

class ICP_matlab
{
public:
    ICP_matlab (Mat M, Mat P);
    ~ICP_matlab();
private:
    Mat get_correspondence_indices(Mat& P, Mat& Q);
};


#endif //GPGPU_ICP_H
