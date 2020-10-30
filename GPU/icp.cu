#include "icp.cuh"

icp::icp(const Mat& src, const Mat& ref)
    : src_(src)
    , ref_(ref)
    , src_transformed_(src.copy())
    , translation_scalars_(Mat(1, 3))
{
    this->rotation_matrix_ = (Mat**)calloc(3, sizeof(Mat*));
    for (int i = 0; i < 3; i++)
        this->rotation_matrix_[i] = new Mat(3, 3);
}

icp::~icp() {
    for (int i = 0; i < 3; i++)
        delete rotation_matrix_[i];
    free(this->rotation_matrix_);
}

void icp::set_rotation_matrix(const float theta1, const float theta2, const float theta3) const {
    rotation_matrix_[0]->m_buffer[0] = 1;
    rotation_matrix_[0]->m_buffer[4] = std::cos(theta1);
    rotation_matrix_[0]->m_buffer[5] = -std::sin(theta1);
    rotation_matrix_[0]->m_buffer[7] = std::sin(theta1);
    rotation_matrix_[0]->m_buffer[8] = std::cos(theta1);

    rotation_matrix_[1]->m_buffer[0] = std::cos(theta2);
    rotation_matrix_[1]->m_buffer[2] = std::sin(theta2);
    rotation_matrix_[1]->m_buffer[4] = 1;
    rotation_matrix_[1]->m_buffer[6] = -std::sin(theta2);
    rotation_matrix_[1]->m_buffer[8] = std::cos(theta2);

    rotation_matrix_[2]->m_buffer[0] = std::cos(theta3);
    rotation_matrix_[2]->m_buffer[1] = -std::sin(theta3);
    rotation_matrix_[2]->m_buffer[3] = std::sin(theta3);
    rotation_matrix_[2]->m_buffer[4] = std::cos(theta3);
    rotation_matrix_[2]->m_buffer[8] = 1;
}

Mat** icp::get_dr(const float theta1, const float theta2, const float theta3) const {
    Mat** res = (Mat**)calloc(3, sizeof(Mat*));
    for (int i = 0; i < 3; i++)
        res[i] = new Mat(3, 3);

    res[0]->m_buffer[4] = -std::sin(theta1);
    res[0]->m_buffer[5] = -std::cos(theta1);
    res[0]->m_buffer[7] = std::cos(theta1);
    res[0]->m_buffer[8] = -std::sin(theta1);

    res[1]->m_buffer[0] = -std::sin(theta2);
    res[1]->m_buffer[2] = std::cos(theta2);
    res[1]->m_buffer[6] = -std::cos(theta2);
    res[1]->m_buffer[8] = -std::sin(theta2);

    res[2]->m_buffer[0] = -std::sin(theta3);
    res[2]->m_buffer[1] = -std::cos(theta3);
    res[2]->m_buffer[3] = std::cos(theta3);
    res[2]->m_buffer[4] = -std::sin(theta3);

    return res;
}

Mat icp::get_jacobian(const Mat& x, const Mat& p_point) const {
    Mat jacobian = Mat(3, 6);
    jacobian.m_buffer[0] = 1;
    jacobian.m_buffer[7] = 1;
    jacobian.m_buffer[14] = 1;

    set_rotation_matrix(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
    auto dr = this->get_dr(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);

    auto jacob1 = dr[0]->dot(*rotation_matrix_[1]).dot(*rotation_matrix_[2]).dot(p_point);
    auto jacob2 = rotation_matrix_[0]->dot(*dr[1]).dot(*rotation_matrix_[2]).dot(p_point);
    auto jacob3 = rotation_matrix_[0]->dot(*rotation_matrix_[1]).dot(*dr[2]).dot(p_point);

    for (int i = 0; i < 3; i++) {
        jacobian.m_buffer[i * jacobian.m_width + 3] = jacob1.m_buffer[i * jacob1.m_width + 0];
        jacobian.m_buffer[i * jacobian.m_width + 4] = jacob2.m_buffer[i * jacob2.m_width + 0];
        jacobian.m_buffer[i * jacobian.m_width + 5] = jacob3.m_buffer[i * jacob3.m_width + 0];
    }


    for (int i = 0; i < 3; i++)
        delete dr[i];
    free(dr);
    return jacobian;
}

Mat icp::err(const Mat& x, const Mat& p_point, const Mat& q_point) const {
    set_rotation_matrix(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
    auto translation = Mat(3, 1);
    translation.m_buffer[0] = x.m_buffer[0];
    translation.m_buffer[1] = x.m_buffer[1];
    translation.m_buffer[2] = x.m_buffer[2];
    auto prediction = rotation_matrix_[0]->dot(*rotation_matrix_[1])
        .dot(*rotation_matrix_[2]).dot(p_point) + translation;
    return prediction - q_point;
}

prep_sys_t* icp::prepare_system(Mat& x, Mat& P, Mat& Q) const {
    Mat H = Mat(6,6);
    Mat G = Mat(6,1);
    float chi = 0.;
    for (int i = 0; i < P.m_height; ++i) {
        auto p_width = P.m_width;
        auto q_width = Q.m_width;

        Mat p_point(p_width, 1);
        Mat q_point(q_width, 1);
        for (int j = 0; j < p_width; j++) {
            p_point.m_buffer[j] = P.m_buffer[i * p_width + j];
        }
        for (int j = 0; j < q_width; j++) {
            q_point.m_buffer[j] = Q.m_buffer[i * q_width + j];
        }

        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        H = H + J.T().dot(J);
        G = G + J.T().dot(e);
        chi += e.T().dot(e).m_buffer[0];
    }
    prep_sys_t* res = (prep_sys_t*)malloc(sizeof(prep_sys_t));
    res->h = new Mat(H);
    res->g = new Mat(G);
    res->chi = chi;
    return res;
}

// Compute the 3 rotation matrix and the 3 translation scalars to transform src_ in ref_
icp& icp::fit(int iterations, float threshold, bool force_iteration){
    auto x = Mat(1,6); // 3 rotation factors + 3 translation
    float chi = 0.;
    int i = 0;
    for (; i < iterations; ++i){
        set_rotation_matrix(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
        auto prep_sys = prepare_system(x, src_, ref_);
        Mat H = *prep_sys->h;
        Mat G = *prep_sys->g;
        float chi = prep_sys->chi;
        auto dx = H.inverse().dot(G).T();
        x = x - dx;
        translation_scalars_ = Mat(1, 3);
        translation_scalars_.m_buffer[0] = x.m_buffer[0];
        translation_scalars_.m_buffer[1] = x.m_buffer[1];
        translation_scalars_.m_buffer[2] = x.m_buffer[2];
        src_transformed_ = rotation_matrix_[0]->dot(*rotation_matrix_[1])
            .dot(*rotation_matrix_[2]).dot(src_.T()).T() + translation_scalars_;
        delete prep_sys->h;
        delete prep_sys->g;
        free(prep_sys);
        if (chi < threshold && not force_iteration)
            break;
    }
    if (chi >= threshold)
        std::cerr << "ICP did not converge in " << iterations << " iterations, and have a chi value of " << chi << "\n";
    else
        std::cerr << "ICP converge in " << i << " iterations, and have a chi value of " << chi << "\n";

    return *this;
}
