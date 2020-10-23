#include "icp.hh"

icp::icp(int n_dim)
{
    if (n_dim != 2 && n_dim != 3) {
        std::cerr << "dim must be 2 or 3\n";
        return;
    }
    this->_n_dim = n_dim;
}


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

// For each point in P find closest one in Q.
Correspondences get_correspondence_indices(const Mat& P, const Mat& Q) {
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

Mat icp::err(const Mat& x, const Mat& p_point, const Mat& q_point) const {
    auto rotation = this->get_r(x[0][3], x[0][4], x[0][5]);
    auto translation = Mat(std::vector<float>{x[0][0], x[0][1], x[0][2]});
    auto prediction = rotation[0].dot(rotation[1]).dot(rotation[2]).dot(p_point) + translation;
    return prediction - q_point;
}

icp::prep_sys_t icp::prepare_system(Mat& x, Mat& P, Mat& Q, Correspondences& corr) const {
    Mat H = Mat(6,6);
    Mat G = Mat(6,1);
    float chi = 0.;
    for (auto elm : corr){
        Mat p_point = Mat({P[std::get<0>(elm)]});
        Mat q_point = Mat({Q[std::get<1>(elm)]});
        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        H = H + J.T().dot(J);
        G = G + J.T().dot(e);
        chi += e.T().dot(e)[0][0];
    }
    auto res = icp::prep_sys_t{H,G,chi};
    return res;
}

Mat icp::icp_least_squares(Mat P, Mat Q){
    int iterations = 30;
    auto x = Mat(1,6); // 3D point + angle?
    //std::vector<float> chi_values;
    //std::vector<Mat> x_values = {x.copy()};
    //std::vector<Mat> P_values = {P.copy()};
    auto P_copy = P.copy();
    // std::vector<Correspondences> corresp_values;
    for (int i = 0; i < iterations; ++i){
        auto rotation = get_r(x[0][3], x[0][4], x[0][5]);
        std::vector<float> x_coords = {x[0][0], x[0][1], x[0][2]};
        auto correspondences = get_correspondence_indices(P_copy, Q);
        //corresp_values.push_back(correspondences);
        auto prep_sys = prepare_system(x, P, Q, correspondences);
        auto H = std::get<0>(prep_sys);
        auto G = std::get<1>(prep_sys);
        //auto chi = std::get<2>(prep_sys);
        auto dx = H.inverse().dot(G) * -1;
        x = x + dx.T();
        //x[0][3] = atan2(sin(x[0][3]), cos(x[0][3]));
        //chi_values.push_back(chi);
        //x_values.push_back(x.copy());
        auto translation = Mat(std::vector<float>{x[0][0], x[0][1], x[0][2]}).T();
        P_copy = rotation[0].dot(rotation[1]).dot(rotation[2]).dot(P.T()).T() + translation;
    }
    //corresp_values.push_back(corresp_values[corresp_values.size() - 1]);
    return P_copy;
}

/*
def icp_least_squares(P, Q, iterations=30, kernel=lambda distance: 1.0):
    x = np.zeros((3, 1))
    chi_values = []
    x_values = [x.copy()]  # Initial value for transformation.
    P_values = [P.copy()]
    P_copy = P.copy()
    corresp_values = []
    for i in range(iterations):
        rot = R(x[2])
        t = x[0:2]
        correspondences = get_correspondence_indices(P_copy, Q)
        corresp_values.append(correspondences)
        H, g, chi = prepare_system(x, P, Q, correspondences, kernel)
        dx = np.linalg.lstsq(H, -g, rcond=None)[0]
        x += dx
        x[2] = atan2(sin(x[2]), cos(x[2])) # normalize angle
        chi_values.append(chi.item(0))
        x_values.append(x.copy())
        rot = R(x[2])
        t = x[0:2]
        P_copy = rot.dot(P.copy()) + t
        P_values.append(P_copy)
    corresp_values.append(corresp_values[-1])
    return P_values, chi_values, corresp_values
*/
