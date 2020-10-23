#include "icp-matlab.hh"

float norm(std::vector<float> a, std::vector<float> b) {
    float r = 0;
    for (int i = 0; i < a.size(); i++) {
        r += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(r);
}

// For each point in P find closest one in Q.
Mat ICP_matlab::get_correspondences(const Mat& P, const Mat& M) {
    Mat Y(P.m_height, P.m_width);

    for (int i = 0; i < P.m_height; ++i)
    {
        std::vector<float> d;
        for (int k = 0; k < M.m_height; ++k)
            d.push_back(norm(P[i], M[k]));
        auto min_idx = std::min_element(d.begin(), d.end()) - d.begin();
        for (int j = 0; j < M[min_idx].size(); ++j)
            Y[i][j] = M[min_idx][j];
    }

    return Y;
}


// M is the model points
// P is points that need to be transformed
ICP_matlab::ICP_matlab (const Mat& M, const Mat& P)
    : dim_(3)
    , scaling_factor_(1)
    , rotation_matrix_(Mat::eye(dim_))
    , translation_offset_(Mat(dim_, 1))
    , p_transformed_(P)
    , err_(0.)
{
    long nb_p_point = P.m_height;
    // long nb_m_point = M.m_height;
    int max_iter = 1000;
    double treshold = 0.000001;
    int i = 0;

    for (; i < max_iter; ++i) {
        auto Y = get_correspondences(P, M);
        err_ = find_alignment(Y, p_transformed_);
        for (long j = 0; j < nb_p_point; ++j)
        {
            auto new_point = rotation_matrix_.dot(p_transformed_[j]) * scaling_factor_ + translation_offset_;
            for (int k = 0; k < new_point[0].size(); ++k)
                p_transformed_[j][k] = new_point[0][k];
            Mat e = Mat(Y[j]) - Mat(p_transformed_[j]);
            err_ += (e.T().dot(e))[0][0];
        }
        err_ /= nb_p_point;

        if (err_ < treshold)
            break;
    }

    if (i >= max_iter) {
        std::cerr << "ICP did not converge in max iter = " << max_iter << " operations, error = " << err_ << "\n";
    }
}

float sum_multiply(const std::vector<float>& a, const std::vector<float>& b)
{
    float ret = 0;
    for (int i = 0; i < a.size(); ++i)
        ret += a[i] * b[i];
    return ret;
}

float ICP_matlab::find_alignment(const Mat& Y, const Mat& P)
{
    long nb_p_point = P.m_height;
    long nb_y_point = Y.m_height;

    if (nb_p_point != nb_y_point)
        throw "Point sets need to have same number of points";
    if (Y.m_width != 3 && P.m_width != 3)
        throw "Need points of dimension3";
    if (nb_p_point < 4)
        throw "Need at least 4 points";

    auto mean_p = P.mean();
    auto mean_y = Y.mean();

    auto pprime = P - mean_p;
    auto yprime = Y - mean_y;

    auto PT = pprime.T();
    auto px = PT[0];
    auto py = PT[1];
    auto pz = PT[2];

    auto YT = yprime.T();
    auto yx = YT[0];
    auto yy = YT[1];
    auto yz = YT[2];

    auto Sxx = sum_multiply(px, yx);
    auto Sxy = sum_multiply(px, yy);
    auto Sxz = sum_multiply(px, yz);

    auto Syx = sum_multiply(py, yx);
    auto Syy = sum_multiply(py, yy);
    auto Syz = sum_multiply(py, yz);

    auto Szx = sum_multiply(pz, yx);
    auto Szy = sum_multiply(pz, yy);
    auto Szz = sum_multiply(pz, yz);

    auto Nmatrix = Mat({
            {Sxx + Syy + Szz, Syz - Szy, -Sxz + Szx, Sxy - Syx},
            {-Szy + Syz, Sxx - Szz - Syy, Sxy + Syx, Sxz + Szx},
            {Szx - Sxz, Syx + Sxy, Syy - Szz - Sxx, Syz + Szy},
            {-Syx + Sxy, Szx + Sxz, Szy + Syz, Szz - Syy - Sxx}
        });

    //TODO: Compute eigen vector and value
    auto eig = Nmatrix.eigen();
    auto q = std::get<1>(eig[eig.size() - 1]); // eigen vector associated to largest eigen value

    auto Qbar = Mat({
            {q[0], -q[1], -q[2], -q[3]},
            {q[1], q[0], q[3], -q[2]},
            {q[2], -q[3], q[0], q[1]},
            {q[3], q[2], -q[1], q[0]}
        });

    auto Q = Mat({
            {q[0], -q[1], -q[2], -q[3]},
            {q[1], q[0], -q[3], q[2]},
            {q[2], q[3], q[0], -q[1]},
            {q[3], -q[2], q[1], q[0]}
        });

    auto rot = Qbar.T().dot(Q);
    // R = R(2:4, 2:4)
    for (int i = 0; i < rot.m_height - 1; ++i)
        for (int j = 0; j < rot.m_width - 1; ++j)
            rotation_matrix_[i][j] = rot[i + 1][j + 1];

    // Compute scaling factor
    float sp = 0;
    float d = 0;
    for (int i = 0; i < nb_p_point; ++i)
    {
        d += Mat(yprime[i]).T().dot(yprime[i])[0][0];
        sp += Mat(pprime[i]).T().dot(pprime[i])[0][0];
    }
    scaling_factor_ = std::sqrt(d / sp);

    // Compute translational offset
    translation_offset_ = mean_y.T() - rotation_matrix_.dot(mean_p.T()) * scaling_factor_;

    // Compute residual error
    float err = 0.;
    for (int i = 0; i < nb_p_point; ++i)
    {
        auto d = Mat(Y[i]) - rotation_matrix_.dot(P[i]) * scaling_factor_ + translation_offset_;
        err += d.T().dot(d)[0][0];
    }

    return err;
}
