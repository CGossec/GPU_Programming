#include "icp.hh"

icp::icp(int n_dim)
{
    if (n_dim != 2 && n_dim != 3) {
        std::cerr << "dim must be 2 or 3\n";
        return;
    }
    this->_n_dim = n_dim;
}


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

std::array<MAT3x6, 3> icp::get_jacobian(std::vector<float> x, std::vector<float> p_point) {
    float theta = x[2];
    MAT3x6 jacobianx{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    MAT3x6 jacobiany{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    MAT3x6 jacobianz{{
                        {1, 0, 0, 0, 0, 0},
                        {0, 1, 0, 0, 0, 0},
                        {0, 0, 1, 0, 0, 0}
                    }};
    auto dr = this->get_dr(theta);
    for (int i = 0; i < 3; i++) {
        for (int k = 0; k < 3; k++) {
            jacobianx[i][k + 3] = dr[0][i][k] * p_point[k];
            jacobiany[i][k + 3] = dr[1][i][k] * p_point[k];
            jacobianz[i][k + 3] = dr[2][i][k] * p_point[k];
        }
    }
    std::array<MAT3x6, 3> jacobians {jacobianx, jacobiany, jacobianz};
    return jacobians;
}



float norm(Point3D p) {
    float r = 0;
    for (int i = 0; i < 3; i++) {
        r += p[i]*p[i];
    }
    return std::sqrt(r);
}

Point3D operator- (const Point3D& first, const Point3D& second){
    Point3D res;
    for (int i = 0; i < 3; i++){
        res[i] = first[i] - second[i];
    }
    return res;
}

Point3D operator+ (const Point3D& first, const Point3D& second){
    Point3D res;
    for (int i = 0; i < 3; i++){
        res[i] = first[i] + second[i];
    }
    return res;
}

// For each point in P find closest one in Q.
Correspondences get_correspondence_indices(std::vector<Point3D> P, std::vector<Point3D> Q) {;
    Correspondences correspondences;
    for (std::size_t i = 0; i < P.size(); i++) {
        auto p_point = P[i];
        auto min_dist = std::numeric_limits<float>::max();
        int chosen_idx = -1;
        for (std::size_t j = 0; j < Q.size(); j++) {
            auto q_point = Q[j];
            auto dist = norm(p_point - q_point);
            if (dist < min_dist){
                min_dist = dist;
                chosen_idx = j;
            }
        }
        correspondences.push_back(std::tuple<std::size_t, std::size_t>{i, chosen_idx});
    }
    return correspondences;
}

/*
float icp::err(std::vector<float> x, Point3D p_point, Point3D q_point){
    auto rotation = this->get_r(x[2]);
    auto translation = x[0:2];
    auto prediction = ;//TODO
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
        auto p_point = std::at<0>(elm);
        auto q_point = std::at<1>(elm);
    }
}*/