#include "icp.hh"
#include "matrices.hh"

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



float norm(Point3D p) {
    float r = 0;
    for (int i = 0; i < 3; i++) {
        r += p[i]*p[i];
    }
    return std::sqrt(r);
}

// For each point in P find closest one in Q.
Correspondences get_correspondence_indices(Mat P, Mat Q) {;
    Correspondences correspondences;
    for (std::size_t i = 0; i < P.m_height; i++) {
        auto p_point = Mat({P[i]});
        auto min_dist = std::numeric_limits<float>::max();
        int chosen_idx = -1;
        for (std::size_t j = 0; j < Q.m_height; j++) {
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

float icp::err(Mat x, Mat p_point, Mat q_point){
    auto rotation = this->get_r(x[0][2]);
    auto translation = x[0];
    auto prediction = rotation.dot(p_point) + translation;
    return prediction - q_point;
}

void icp::prepare_system(Point3D x, std::vector<Point3D> P, std::vector<Point3D> Q, Correspondences corr){
    MAT3x3 h1 = {{
                         {0,0,0},
                         {0,0,0},
                         {0,0,0},
                 }};
    MAT3x3 h2 = {{
                         {0,0,0},
                         {0,0,0},
                         {0,0,0},
                 }};
    MAT3x3 h3 = {{
                         {0,0,0},
                         {0,0,0},
                         {0,0,0},
                 }};
    std::array<float, 3> g1 = {{0,0,0}};
    std::array<float, 3> g2 = {{0,0,0}};
    std::array<float, 3> g3 = {{0,0,0}};
    float chi =0;
    for (auto elm : corr){
        auto p_point = P.at(std::get<0>(elm));
        auto q_point = Q.at(std::get<1>(elm));
        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        h1 = h1 + dot_transpose(J.at(0));
        h2 = h2 + dot_transpose(J.at(1));
        h3 = h3 + dot_transpose(J.at(2));
        g1 = g1 + transpose(J.at(0));
        g2 = g2 + transpose(J.at(1));
        g3 = g3 + transpose(J.at(2));
    }
}