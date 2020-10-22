#import "icp-matlab.hh"

// For each point in P find closest one in Q.
Mat ICP_matlab::get_correspondence_indices(Mat& P, Mat& Q) {
    Mat correspondences(P.m_height, 2);
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
        //correspondences.push_back(std::tuple<std::size_t, std::size_t>{i, chosen_idx});
        correspondences[i] = {i, chosen_idx};
    }
    return correspondences;
}

// M is the model points
// P is points that need to be transformed
ICP_matlab::ICP_matlab (Mat M, Mat P)
{
    long nb_p_point = P.m_width;
    long nb_m_point = M.m_width;
    int dim = 3;
    int scaling_factor = 1;
    int max_iter = 200;
    double treshold = 0.0001;

    Mat rotation_matrix Mat::eye(dim);
    Mat translation_offset(dim, 1);
    Mat p_copy(P);

    for (int i = 0; i < max_iter; ++i) {
        
    }

}
