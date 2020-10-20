
#ifndef GPGPU_ICP_H
#define GPGPU_ICP_H

#include <array>
#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>

typedef std::array<float, 3> Point3D;
typedef std::vector<std::tuple<std::size_t, std::size_t>> Correspondences;
typedef std::array<std::array<float, 3>, 3> MAT3x3;
typedef std::array<std::array<float, 6>, 3> MAT3x6;
typedef std::array<std::array<float, 3>, 6> MAT6x3;

class icp
{
public:
    icp(int n_dim);
    ~icp();
private:
    int _n_dim;

    std::array<MAT3x3, 3> get_r(float theta);
    std::array<MAT3x3, 3> get_dr(float theta);
    std::array<MAT3x6, 3> get_jacobian(std::vector<float> x, std::vector<float> p_point);
    void prepare_system(Point3D x, std::vector<Point3D> P, std::vector<Point3D> Q, Correspondences corr);
    float err(std::vector<float> x, Point3D p_point, Point3D q_point);
};
Correspondences get_correspondence_indices(std::vector<Point3D> P, std::vector<Point3D> Q);
MAT3x3 dot_transpose(MAT3x6 mat);
#endif //GPGPU_ICP_H
