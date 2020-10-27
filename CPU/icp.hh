
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
    Mat icp_least_squares(Mat P, Mat Q);

private:
    int _n_dim;

    using prep_sys_t = std::tuple<Mat, Mat, float>;

    std::array<Mat, 3> get_r(const float theta1, const float theta2, const float theta3) const;
    std::array<Mat, 3> get_dr(const float theta1, const float theta2, const float theta3) const;
    Mat get_jacobian(const Mat&, const Mat&) const;
    Mat err(const Mat& x, const Mat& p_point, const Mat& q_point) const;
    prep_sys_t prepare_system(Mat& x, Mat& P, Mat& Q, Correspondences& corr) const;
};
Correspondences get_correspondence_indices(const Mat& P, const Mat& Q);

#endif //GPGPU_ICP_H
