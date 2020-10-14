#include <array>
#include <cmath>
#include <vector>
#include <iostream>

class icp
{
public:
    icp(int n_dim);
    ~icp();
private:
    int _n_dim;

    std::array<MAT3x3, 3> get_r(float theta);
    std::array<MAT3x3, 3> get_dr(float theta);
    std::vector<std::vector<float> > get_jacobian(std::vector<float> x, std::vector<float> p_point);
};

icp::icp(int n_dim)
{
    if (n_dim != 2 && n_dim != 3) {
        std::cerr << "dim must be 2 or 3\n";
        return;
    }
    this->_n_dim = n_dim;
}

typedef std::array<std::array<float, 3>, 3>type MAT3x3;
typedef std::array<std::array<float, 3>, 6>type MAT3x6;

std::array<MAT3x3, 3> icp::get_r(float theta) {
    MAT3x3 rx{{
                      {1, 0, 0},
                      {0, std::cos(theta), -std::sin(theta)},
                      {0, std::sin(theta), std::cos(theta),
                      }}};
    MAT3x3 ry{{
                      {std::cos(theta), 0, std::sin(theta)},
                      {0, 1, 0},
                      {-std::sin(theta), 0, -std::cos(theta),
                      }}};
    MAT3x3 rz{{
                      {std::cos(theta), -std::sin(theta), 0},
                      {std::sin(theta), std::cos(theta), 0},
                      {0, 0, 1,
                      }}};
    std::array<MAT3x3, 3> res {rx, ry, rz};
    return res;
}

std::array<MAT3x3, 3> icp::get_dr(float theta) {
    MAT3x3 rx{{
                      {0, 0, 0},
                      {0, -std::sin(theta), -std::cos(theta)},
                      {0, std::cos(theta), -std::sin(theta),
                      }}};
    MAT3x3 ry{{
                      {-std::sin(theta), 0, std::cos(theta)},
                      {0, 0, 0},
                      {-std::cos(theta), 0, -std::sin(theta),
                      }}};
    MAT3x3 rz{{
                      {-std::sin(theta), -std::cos(theta), 0},
                      {std::cos(theta), -std::sin(theta), 0},
                      {0, 0, 0,
                      }}};
    std::array<MAT3x3, 3> res {rx, ry, rz};
    return res;
}

std::vector<std::vector<float> > icp::get_jacobian(std::vector<float> x, std::vector<float> p_point) {
    float theta = x[2];
    MAT3x6 jacobianx{{{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}}};
    MAT3x6 jacobiany{{{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}}};
    MAT3x6 jacobianz{{{1, 0, 0, 0, 0, 0}, {0, 1, 0, 0, 0, 0}, {0, 0, 1, 0, 0, 0}}};
    auto dr = this->get_dr(theta);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            jacobianx[i][k + 3] = dr[0][i][k] * p_point[k];
            jacobiany[i][k + 3] = dr[1][i][k] * p_point[k];
            jacobianz[i][k + 3] = dr[2][i][k] * p_point[k];
        }
    }
    std::array<MAT3x6, 3> jacobians {jacobianx, jacobiany, jacobianz};
    return jacobian;
}
