#include "icp.hh"

icp::icp(const Mat& src, const Mat& ref)
    : src_(src)
    , ref_(ref)
    , src_transformed_(src.copy())
    , translation_scalars_(Mat(1, 3)) 
{
    this->rotation_matrix_ = (Mat*)calloc(3, sizeof(Mat));
    for (int i = 0; i < 3; i++)
        this->rotation_matrix_[i] = Mat(3, 3);
}

icp::~icp() {
    free(this->rotation_matrix_);
}

Mat* icp::get_r(const float theta1, const float theta2, const float theta3) const {
    Mat* res = (Mat*)calloc(3, sizeof(Mat));
    
    float *vx = (float*)calloc(9, sizeof(float));
    vx[0] = 1;
    vx[4] = std::cos(theta1);
    vx[5] = -std::sin(theta1);
    vx[7] = std::sin(theta1);
    vx[8] = std::cos(theta1);
    res[0] = Mat(vx, 3, 3);

    float *vy = (float*)calloc(9, sizeof(float));
    vy[0] = std::cos(theta2);
    vy[2] = std::sin(theta2);
    vy[4] = 1;
    vy[6] = -std::sin(theta2);
    vy[8] = std::cos(theta2);
    res[1] = Mat(vy, 3, 3);

    float *vz = (float*)calloc(9, sizeof(float));
    vz[0] = std::cos(theta3);
    vz[1] = -std::sin(theta3);
    vz[3] = std::sin(theta3);
    vz[4] = std::cos(theta3);
    vz[8] = 1;
    res[2] = Mat(vz, 3, 3);

    return res;
}

Mat* icp::get_dr(const float theta1, const float theta2, const float theta3) const {
    Mat* res = (Mat*)calloc(3, sizeof(Mat));
    
    float *vx = (float*)calloc(9, sizeof(float));
    vx[4] = -std::sin(theta1);
    vx[5] = -std::cos(theta1);
    vx[7] = std::cos(theta1);
    vx[8] = -std::sin(theta1);
    res[0] = Mat(vx, 3, 3);

    float *vy = (float*)calloc(9, sizeof(float));
    vy[0] = -std::sin(theta2);
    vy[2] = std::cos(theta2);
    vy[6] = -std::cos(theta2);
    vy[8] = -std::sin(theta2);
    res[1] = Mat(vy, 3, 3);

    float *vz = (float*)calloc(9, sizeof(float));
    vz[0] = -std::sin(theta3);
    vz[1] = -std::cos(theta3);
    vz[3] = std::cos(theta3);
    vz[4] = -std::sin(theta3);
    res[2] = Mat(vz, 3, 3);

    return res;
}

Mat icp::get_jacobian(const Mat& x, const Mat& p_point) const {
    float* v = (float*)calloc(3*6, sizeof(float));
    v[0] = 1;
    v[7] = 1;
    v[14] = 1;
    Mat jacobian = Mat(v, 3, 6);
    
    auto r = this->get_r(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
    auto dr = this->get_dr(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);

    auto jacob1 = dr[0].dot(r[1]).dot(r[2]).dot(p_point);
    auto jacob2 = r[0].dot(dr[1]).dot(r[2]).dot(p_point);
    auto jacob3 = r[0].dot(r[1]).dot(dr[2]).dot(p_point);

    for (int i = 0; i < 3; i++) {
        jacobian.m_buffer[i * jacobian.m_width + 3] = jacob1.m_buffer[i * jacob1.m_width + 0];
        jacobian.m_buffer[i * jacobian.m_width + 4] = jacob2.m_buffer[i * jacob2.m_width + 0];
        jacobian.m_buffer[i * jacobian.m_width + 5] = jacob3.m_buffer[i * jacob3.m_width + 0];
    }

    return jacobian;
}



// float norm(const Mat& p, int size) {
//     float r = 0;
//     for (int i = 0; i < size; i++) {
//         r += p.m_buffer[i] * p.m_buffer[i];
//     }
//     return std::sqrt(r);
// }

Mat icp::err(const Mat& x, const Mat& p_point, const Mat& q_point) const {
    auto rotation = this->get_r(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
    float* vt = (float*)calloc(3, sizeof(float));
    vt[0] = x.m_buffer[0];
    vt[1] = x.m_buffer[1];
    vt[2] = x.m_buffer[2];
    auto translation = Mat(vt, 3);
    auto prediction = rotation[0].dot(rotation[1]).dot(rotation[2]).dot(p_point) + translation;
    return prediction - q_point;
}

prep_sys_t icp::prepare_system(Mat& x, Mat& P, Mat& Q) const {
    Mat H = Mat(6,6);
    Mat G = Mat(6,1);
    float chi = 0.;
    for (int i = 0; i < P.m_height; ++i) {
        auto p_width = P.m_width;
        auto q_width = Q.m_width;
        float* vp = (float*)calloc(p_width, sizeof(float));
        float* vq = (float*)calloc(q_width, sizeof(float));
        for (int j = 0; j < p_width; j++) {
            vp[j] = P.m_buffer[i * p_width + j];
        }
        for (int j = 0; j < q_width; j++) {
            vq[j] = Q.m_buffer[i * q_width + j];
        }
        Mat p_point = Mat(vp, p_width);
        Mat q_point = Mat(vq, q_width);
        auto e = this->err(x, p_point, q_point);
        auto J = this->get_jacobian(x, p_point);
        H = H + J.T().dot(J);
        G = G + J.T().dot(e);
        chi += e.T().dot(e).m_buffer[0];
    }
    prep_sys_t* res = (prep_sys_t*)calloc(1, sizeof(prep_sys_t));
    res->h = H;
    res->g = G;
    res->chi = chi;
    return res;
}

// Compute the 3 rotation matrix and the 3 translation scalars to transform src_ in ref_
icp& icp::fit(int iterations, float threshold){
    auto x = Mat(1,6); // 3 rotation factors + 3 translation
    float chi = 0.;
    int i = 0;
    for (; i < iterations; ++i){
        rotation_matrix_ = get_r(x.m_buffer[3], x.m_buffer[4], x.m_buffer[5]);
        auto prep_sys = prepare_system(x, src_, ref_);
        auto H = prep_sys.h;
        auto G = prep_sys.g;
        chi = prep_sys.chi;
        auto dx = H.inverse().dot(G).T();
        x = x - dx;
        float* v = (float*)calloc(3, sizeof(float));
        v[0] = x.m_buffer[0];
        v[1] = x.m_buffer[1];
        v[2] = x.m_buffer[2];
        translation_scalars_ = Mat(v, 3).T();
        auto r_width = rotation_matrix_.m_width;
        float* vr0 = (float*)calloc(r_width, sizeof(float));
        float* vr1 = (float*)calloc(r_width, sizeof(float));
        float* vr2 = (float*)calloc(r_width, sizeof(float));

        for (int i = 0; i < r_width; i++) {
            vr0[i] = rotation_matrix_[i];
            vr1[i] = rotation_matrix_[r_width + i];
            vr2[i] = rotation_matrix_[2 * r_width + i];
        }
        Mat r0 = Mat(vr0, r_width);
        Mat r1 = Mat(vr1, r_width);
        Mat r2 = Mat(vr2, r_width);
        src_transformed_ = r0.dot(r1).dot(r2).dot(src_.T()).T() + translation_scalars_;
        if (chi < threshold)
            break;
    }
    if (chi >= threshold)
        std::cerr << "ICP did not converge in " << iterations << " iterations, and have a chi value of " << chi << "\n";
    else
        std::cerr << "ICP converge in " << i << " iterations, and have a chi value of " << chi << "\n";

    return *this;
}
