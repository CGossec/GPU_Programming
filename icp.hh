
#ifndef GPGPU_ICP_H
#define GPGPU_ICP_H

#include <array>
#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>

typedef std::vector<std::tuple<std::size_t, std::size_t>> Correspondences;

class icp
{
public:
    icp(int n_dim);
    ~icp();
private:
    int _n_dim;

    std::array<Mat, 3> get_r(float theta);
    std::array<Mat, 3> get_dr(float theta);
    std::array<Mat, 3> get_jacobian(Mat, Mat);
    void prepare_system(Mat x, Mat P, Mat Q, Correspondences corr);
    float err(Mat x, Mat p_point, Mat q_point);
};
Correspondences get_correspondence_indices(Mat P, Mat Q);

#endif //GPGPU_ICP_H
