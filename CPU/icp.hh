
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
    icp(const Mat& src, const Mat& ref)
        : src_(src)
        , ref_(ref)
        , src_transformed_(src.copy())
        , translation_scalars_(Mat(1, 3))
        , rotation_matrix_(std::array<Mat, 3>{Mat(3, 3), Mat(3, 3), Mat(3, 3)})
        {}
    icp& fit(int iterations = 30, float treshold = 0.0001, bool force_iteration = false);

    Mat get_src_transformed() {
        return src_transformed_;
    }

    Mat get_translation_scalars() {
        return translation_scalars_;
    }

    std::array<Mat, 3> get_rotation_matrix() {
        return rotation_matrix_;
    }

private:
    using prep_sys_t = std::tuple<Mat, Mat, float>;

    std::array<Mat, 3> get_r(const float theta1, const float theta2, const float theta3) const;
    std::array<Mat, 3> get_dr(const float theta1, const float theta2, const float theta3) const;
    Mat get_jacobian(const Mat&, const Mat&) const;
    Mat err(const Mat& x, const Mat& p_point, const Mat& q_point) const;
    prep_sys_t prepare_system(Mat& x, Mat& P, Mat& Q) const;

    Mat src_;
    Mat ref_;
    Mat src_transformed_;
    Mat translation_scalars_;
    std::array<Mat, 3> rotation_matrix_;
};

#endif //GPGPU_ICP_H
