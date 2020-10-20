#include "icp.hh"

icp::icp(int n_dim)
{
    if (n_dim != 2 && n_dim != 3) {
        std::cerr << "dim must be 2 or 3\n";
        return;
    }
    this->_n_dim = n_dim;
}


std::array<Mat, 3> icp::get_r(float theta) {
    Mat rx{{
                      {1, 0, 0},
                      {0, std::cos(theta), -std::sin(theta)},
                      {0, std::sin(theta), std::cos(theta),
                      }}};
    Mat ry{{
                      {std::cos(theta), 0, std::sin(theta)},
                      {0, 1, 0},
                      {-std::sin(theta), 0, -std::cos(theta),
                      }}};
    Mat rz{{
                      {std::cos(theta), -std::sin(theta), 0},
                      {std::sin(theta), std::cos(theta), 0},
                      {0, 0, 1,
                      }}};
    std::array<Mat, 3> res {rx, ry, rz};
    return res;
}

std::array<Mat, 3> icp::get_dr(float theta) {
    Mat rx{{
                      {0, 0, 0},
                      {0, -std::sin(theta), -std::cos(theta)},
                      {0, std::cos(theta), -std::sin(theta),
                      }}};
    Mat ry{{
                      {-std::sin(theta), 0, std::cos(theta)},
                      {0, 0, 0},
                      {-std::cos(theta), 0, -std::sin(theta),
                      }}};
    Mat rz{{
                      {-std::sin(theta), -std::cos(theta), 0},
                      {std::cos(theta), -std::sin(theta), 0},
                      {0, 0, 0,
                      }}};
    std::array<Mat, 3> res {rx, ry, rz};
    return res;
}

std::array<Mat, 3> icp::get_jacobian(Mat x, Mat p_point) {
    float theta = x[0][2];
    Mat jacobianx{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    Mat jacobiany{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    Mat jacobianz{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    auto dr = this->get_dr(theta);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            jacobianx[i][k + 3] = dr[0][i][k] * p_point[0][k];
            jacobiany[i][k + 3] = dr[1][i][k] * p_point[0][k];
            jacobianz[i][k + 3] = dr[2][i][k] * p_point[0][k];
        }
    }
    std::array<Mat, 3> jacobians {jacobianx, jacobiany, jacobianz};
    return jacobians;
}



float norm(Mat p) {
    float r = 0;
    for (int i = 0; i < 3; i++) {
        r += p[0][i] * p[0][i];
    }
    return std::sqrt(r);
}

// For each point in P find closest one in Q.
Correspondences get_correspondence_indices(Mat P, Mat Q) {;
    Correspondences correspondences;
    for (int i = 0; i < P.m_height; i++) {
        auto p_point = Mat({P[i]});
        auto min_dist = std::numeric_limits<float>::max();
        int chosen_idx = -1;
        for (int j = 0; j < Q.m_height; j++) {
            Mat q_point = Mat({Q[j]});
            auto distance_coords = p_point - q_point;
            auto dist = norm(distance_coords);
            if (dist < min_dist){
                min_dist = dist;
                chosen_idx = j;
            }
        }
        correspondences.push_back(std::tuple<std::size_t, std::size_t>{i, chosen_idx});
    }
    return correspondences;
}

Mat icp::err(Mat x, Mat p_point, Mat q_point){
    auto rotation = this->get_r(x[0][2]);
    auto translation = Mat({x[0]});
    auto prediction = rotation[0].dot(rotation[1].dot(rotation[2].dot(p_point))) + translation;
    return prediction - q_point;
}

std::tuple<std::array<Mat,3>,std::array<Mat,3>,float>
icp::prepare_system(Mat x, Mat P, Mat Q, Correspondences corr){
    Mat h1 = Mat(3,3);
    Mat h2 = Mat(3,3);
    Mat h3 = Mat(3,3);
    Mat g1 = Mat(1,3);
    Mat g2 = Mat(1,3);
    Mat g3 = Mat(1,3);
    float chi = 0;
    for (auto elm : corr){
        Mat p_point = Mat({P[std::get<0>(elm)]});
        Mat q_point = Mat({Q[std::get<1>(elm)]});
        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        h1 = h1 + J[0].dot(J[0].T());
        h2 = h2 + J[1].dot(J[1].T());
        h3 = h3 + J[2].dot(J[2].T());
        g1 = g1 + J[0].dot(e);
        g2 = g2 + J[1].dot(e);
        g3 = g3 + J[2].dot(e);
        chi += e.dot(e.T())[0][0];
    }
    std::array<Mat, 3> H = {h1, h2, h3};
    std::array<Mat, 3> G = {g1, g2, g3};
    auto res = std::tuple<std::array<Mat,3>,std::array<Mat,3>,float>({H,G,chi});
    return res;
}