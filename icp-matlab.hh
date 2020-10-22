#ifndef GPGPU_ICP_MATLAB_H
#define GPGPU_ICP_MATLAB_H

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <tuple>
#include <vector>
#include "matrices.hh"

class ICP_matlab
{
public:
    ICP_matlab (const Mat& M, const Mat& P);

    float get_scaling_factor() {
        return scaling_factor_;
    }

    Mat get_rotation_matrix() {
        return rotation_matrix_;
    }

    Mat get_translation_offset() {
        return translation_offset_;
    }

    Mat get_p_transformed() {
        return p_transformed_;
    }

    double get_err() {
        return err_;
    }

private:
    Mat get_correspondences(const Mat& P, const Mat& M);
    float find_alignment(const Mat& P, const Mat& Y);

    int dim_;
    float scaling_factor_;
    Mat rotation_matrix_;
    Mat translation_offset_;
    Mat p_transformed_;
    double err_;
};


#endif //GPGPU_ICP_MATLAB_H
