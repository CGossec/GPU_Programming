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
ICP_matlab::ICP_matlab (const Mat& M, const Mat& P) {
    long nb_p_point = P.m_width;
    long nb_m_point = M.m_width;
    int dim = 3;
    float scaling_factor = 1;
    int max_iter = 200;
    double treshold = 0.0001;
    double err = 0.;

    Mat rotation_matrix = Mat::eye(dim);
    Mat translation_offset(dim, 1);
    Mat p_copy(P);

    for (int i = 0; i < max_iter; ++i) {
        auto Y = get_correspondences(P, M);
        //find_alignement(p_copy, Y);
        for (long j = 0; j < nb_p_point; ++j)
        {
            auto new_point = rotation_matrix.dot(p_copy[j]) * scaling_factor + translation_offset;
            p_copy[j] = new_point[0];
            Mat e(dim, 1);
            for (int k = 0; k < p_copy[j].size(); ++k)
                e[k][0] = Y[j][k] - p_copy[j][k];
            err += (e.T().dot(e))[0][0];
        }
        err /= nb_p_point;

        if (err < treshold)
            break;
    }

}
