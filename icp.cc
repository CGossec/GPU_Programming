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

    std::vector<std::vector<float> > get_r(float theta);
    std::vector<std::vector<float> > get_dr(float theta);
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

std::vector<std::vector<float> > icp::get_r(float theta) {
    std::vector<std::vector<float> > res{{
            {1, 0, 0},
            {0, std::cos(theta), -std::sin(theta)},
            {0, std::sin(theta), std::cos(theta),
        }}};
    return res;
}

std::vector<std::vector<float> > icp::get_dr(float theta) {
    std::vector<std::vector<float> > res{{
            {-std::sin(theta), -std::cos(theta)}, 
            {std::cos(theta), -std::sin(theta),
        }}};
    return res;
}

std::vector<std::vector<float> > icp::get_jacobian(std::vector<float> x, std::vector<float> p_point) {
    float theta = x[2];
    std::vector<std::vector<float> > jacobian{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::vector<float> mult;
    auto dr = this->get_dr(theta);
    for (int i = 0; i < dr.size(); i++) {
        for (int k = 0; k < dr[i].size(); k++) {
            mult[i] += dr[i][k] * p_point[k];
        }
    }
    int index = 0;
    for (auto r : jacobian) {
        r.push_back(mult[index++]);
    }
    return jacobian;
}
