#import "icp-matlab.hh"

float norm(std::vector<float> a, std::vector<float> b) {
    float r = 0;
    for (int i = 0; i < a.size(); i++) {
        r += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(r);
}

// For each point in P find closest one in Q.
Mat ICP_matlab::get_correspondences(const Mat& P, const Mat& M) {
    Mat Y(P.m_height, P.m_width) ;

    for (int i = 0; i < P.m_width; ++i)
    {
        std::vector<float> d;
        for (int k = 0; k < M.m_width; ++k)
            d.push_back(norm(P[i], M[k]));
        auto min_idx = std::min_element(d.begin(), d.end()) - d.begin();
        Y[i] = M[min_idx];
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
{
    long nb_p_point = P.m_width;
    long nb_m_point = M.m_width;
    int max_iter = 200;
    double treshold = 0.0001;
    double err = 0.;

    Mat p_copy(P);

    for (int i = 0; i < max_iter; ++i) {
        auto Y = get_correspondences(P, M);
        //find_alignement(p_copy, Y);
        for (long j = 0; j < nb_p_point; ++j)
        {
            auto new_point = rotation_matrix_.dot(p_copy[j]) * scaling_factor_ + translation_offset_;
            p_copy[j] = new_point[0];
            Mat e(dim_, 1);
            for (int k = 0; k < p_copy[j].size(); ++k)
                e[k][0] = Y[j][k] - p_copy[j][k];
            err += (e.T().dot(e))[0][0];
        }
        err /= nb_p_point;

        if (err < treshold)
            break;
    }

}

float sum_multiply(const std::vector<float>& a, const std::vector<float>& b)
{
    float ret = 0;
    for (int i = 0; i < a.size(); ++i)
        ret += a[i] * a[i];
    return ret;
}

float ICP_matlab::find_alignment(const Mat& Y, const Mat& P)
{
    long nb_p_point = P.m_width;
    long nb_y_point = Y.m_width;

    if (nb_p_point != nb_y_point)
        throw "Point sets need to have same number of points";
    if (Y.m_height != 3 && P.m_height != 3)
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
    Mat D = Mat::eye(4); // eigen value
    Mat V(4, 4); // eigen vector

    auto q = V[3]; // eigen vector associated to largest eigen value

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

    rotation_matrix_ = Qbar.T().dot(Q);
    // R = R(2:4, 2:4)

    // Compute scaling factor
    float sp = 0;
    float d = 0;
    for (int i =0; i < nb_p_point; ++i)
    {
        d += Mat({yprime[i]}).T().dot(Mat({yprime[i]}))[0][0];
        sp += Mat({pprime[i]}).T().dot(Mat({pprime[i]}))[0][0];
    }
    scaling_factor_ = std::sqrt(d / sp);

    // Compute translational offset
    translation_offset_ = mean_y - rotation_matrix_.dot(mean_p) * scaling_factor_;

    // Compute residual error
    float err = 0.;
    for (int i = 0; i < nb_p_point; ++i)
    {
        auto d = Mat({Y[i]}) - rotation_matrix_.dot(P[i]) * scaling_factor_ + translation_offset_;
        err += d.T().dot(d)[0][0];
    }

    return err;
}
