
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

class icp
{
public:
    icp(int n_dim);
    ~icp();
private:
    int _n_dim;

    using prep_sys_t = std::tuple<std::array<Mat,3>,std::array<Mat,3>,Mat>;

    std::array<Mat, 3> get_r(const float theta) const;
    std::array<Mat, 3> get_dr(const float theta) const;
    std::array<Mat, 3> get_jacobian(const Mat&, const Mat&) const;
    Mat err(const Mat& x, const Mat& p_point, const Mat& q_point) const;
    prep_sys_t prepare_system(Mat& x, Mat& P, Mat& Q, Correspondences& corr) const;
    void icp_least_squares(Mat&, Mat&);
};
Correspondences get_correspondence_indices(Mat& P, Mat& Q);

#endif //GPGPU_ICP_H
