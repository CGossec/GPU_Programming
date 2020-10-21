#include "icp.hh"

icp::icp(int n_dim)
{
    if (n_dim != 2 && n_dim != 3) {
        std::cerr << "dim must be 2 or 3\n";
        return;
    }
    this->_n_dim = n_dim;
}


std::array<Mat, 3> icp::get_r(const float theta) const {
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

std::array<Mat, 3> icp::get_dr(const float theta) const {
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

std::array<Mat, 3> icp::get_jacobian(const Mat& x, const Mat& p_point) const {
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
Correspondences get_correspondence_indices(Mat& P, Mat& Q) {
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
    auto rotation = this->get_r(x[0][2]);
    auto translation = Mat({x[0]});
    auto prediction = rotation[0].dot(rotation[1].dot(rotation[2].dot(p_point))) + translation;
    return prediction - q_point;
}

icp::prep_sys_t icp::prepare_system(Mat& x, Mat& P, Mat& Q, Correspondences& corr) const {
    Mat h1 = Mat(3,3);
    Mat h2 = Mat(3,3);
    Mat h3 = Mat(3,3);
    Mat g1 = Mat(1,3);
    Mat g2 = Mat(1,3);
    Mat g3 = Mat(1,3);
    Mat chi = Mat(3,3);
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
        //chi += e * e.T();
    }
    std::array<Mat, 3> H = {h1, h2, h3};
    std::array<Mat, 3> G = {g1, g2, g3};
    auto res = std::tuple<std::array<Mat,3>,std::array<Mat,3>,Mat>{H,G,chi};
    return res;
}
/*
void icp::icp_least_squares(Mat P, Mat Q){
    int iterations = 30;
    auto x = Mat(1,4); // 3D point + angle?
    std::vector<Mat> chi_values;
    std::vector<Mat> x_values = {x.copy()};
    std::vector<Mat> P_values = {P.copy()};
    auto P_copy = P.copy();
    std::vector<Correspondences> corresp_values;
    for (int i = 0; i < iterations; ++i){
        auto rot = get_r(x[0][3]);
        std::vector<float> x_coords = {x[0][0], x[0][1], x[0][2]};
        auto correspondences = get_correspondence_indices(P_copy, Q);
        corresp_values.push_back(correspondences);
        auto prep_sys = prepare_system(x, P, Q, correspondences);
        auto H = std::get<0>(prep_sys);
        auto G = std::get<1>(prep_sys);
        auto chi = std::get<2>(prep_sys);
        auto dx = np.linalg.lstsq(H, -g, rcond=None)[0]; //FIXME hahahahahahahaha
        x += dx;
        x[0][3] = atan2(sin(x[0][3]), cos(x[0][3]));
        chi_values.push_back(chi[0][0])
    }
}

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
