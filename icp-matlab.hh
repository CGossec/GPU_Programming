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

class icp_matlab
{
public:
    icp(int n_dim);
    ~icp();
private:
};

Correspondences get_correspondence_indices(Mat& P, Mat& Q);

#endif //GPGPU_ICP_H
