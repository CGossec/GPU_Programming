#include "icp.hh"

std::array<Mat, 3> icp::get_r(const float theta1, const float theta2, const float theta3) const {
    Mat rx{{
                      {1, 0, 0},
                      {0, std::cos(theta1), -std::sin(theta1)},
                      {0, std::sin(theta1), std::cos(theta1),
                      }}};
    Mat ry{{
                      {std::cos(theta2), 0, std::sin(theta2)},
                      {0, 1, 0},
                      {-std::sin(theta2), 0, std::cos(theta2),
                      }}};
    Mat rz{{
                      {std::cos(theta3), -std::sin(theta3), 0},
                      {std::sin(theta3), std::cos(theta3), 0},
                      {0, 0, 1,
                      }}};
    std::array<Mat, 3> res {rx, ry, rz};
    return res;
}

std::array<Mat, 3> icp::get_dr(const float theta1, const float theta2, const float theta3) const {
    Mat rx{{
                      {0, 0, 0},
                      {0, -std::sin(theta1), -std::cos(theta1)},
                      {0, std::cos(theta1), -std::sin(theta1),
                      }}};
    Mat ry{{
                      {-std::sin(theta2), 0, std::cos(theta2)},
                      {0, 0, 0},
                      {-std::cos(theta2), 0, -std::sin(theta2),
                      }}};
    Mat rz{{
                      {-std::sin(theta3), -std::cos(theta3), 0},
                      {std::cos(theta3), -std::sin(theta3), 0},
                      {0, 0, 0,
                      }}};
    std::array<Mat, 3> res {rx, ry, rz};
    return res;
}

Mat icp::get_jacobian(const Mat& x, const Mat& p_point) const {
    Mat jacobian{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    auto r = this->get_r(x[0][3], x[0][4], x[0][5]);
    auto dr = this->get_dr(x[0][3], x[0][4], x[0][5]);

    auto jacob1 = dr[0].dot(r[1]).dot(r[2]).dot(p_point);
    auto jacob2 = r[0].dot(dr[1]).dot(r[2]).dot(p_point);
    auto jacob3 = r[0].dot(r[1]).dot(dr[2]).dot(p_point);

    for (int i = 0; i < 3; i++) {
        jacobian[i][3] = jacob1[i][0];
        jacobian[i][4] = jacob2[i][0];
        jacobian[i][5] = jacob3[i][0];
    }

    return jacobian;
}



float norm(const Mat& p) {
    float r = 0;
    for (std::size_t i = 0; i < p[0].size(); i++) {
        r += p[0][i] * p[0][i];
    }
    return std::sqrt(r);
}

Mat icp::err(const Mat& x, const Mat& p_point, const Mat& q_point) const {
    auto rotation = this->get_r(x[0][3], x[0][4], x[0][5]);
    auto translation = Mat(std::vector<float>{x[0][0], x[0][1], x[0][2]});
    auto prediction = rotation[0].dot(rotation[1]).dot(rotation[2]).dot(p_point) + translation;
    return prediction - q_point;
}

icp::prep_sys_t icp::prepare_system(Mat& x, Mat& P, Mat& Q) const {
    Mat H = Mat(6,6);
    Mat G = Mat(6,1);
    float chi = 0.;
    for (int i = 0; i < P.m_height; ++i) {
        Mat p_point = Mat({P[i]});
        Mat q_point = Mat({Q[i]});
        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        H = H + J.T().dot(J);
        G = G + J.T().dot(e);
        chi += e.T().dot(e)[0][0];
    }
    auto res = icp::prep_sys_t{H,G,chi};
    return res;
}

// Compute the 3 rotation matrix and the 3 translation scalars to transform src_ in ref_
icp& icp::fit(int iterations, float treshold){
    auto x = Mat(1,6); // 3 rotation factors + 3 translation
    float chi = 0.;
    int i = 0;
    for (; i < iterations; ++i){
        rotation_matrix_ = get_r(x[0][3], x[0][4], x[0][5]);
        auto prep_sys = prepare_system(x, src_, ref_);
        auto H = std::get<0>(prep_sys);
        auto G = std::get<1>(prep_sys);
        chi = std::get<2>(prep_sys);
        auto dx = H.inverse().dot(G).T();
        x = x - dx;
        translation_scalars_ = Mat(std::vector<float>{x[0][0], x[0][1], x[0][2]}).T();
        src_transformed_ = rotation_matrix_[0].dot(rotation_matrix_[1])
            .dot(rotation_matrix_[2]).dot(src_.T()).T() + translation_scalars_;
        if (chi < treshold)
            break;
    }
    if (chi >= treshold)
        std::cerr << "ICP did not converge in " << iterations << " iterations, and have a chi value of " << chi << "\n";
    else
        std::cerr << "ICP converge in " << i << " iterations, and have a chi value of " << chi << "\n";

    return *this;
}
