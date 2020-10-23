#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include "matrices.hh"
#include "icp.hh"
#include "icp-matlab.hh"

Mat parse_file(std::ifstream &file) {
    std::string line;
    int counter = 0;
    std::vector<std::vector<float> > results;
    while (std::getline(file, line))
    {
        if (!counter) {
            counter++;
            continue;
        }
        std::vector<float> coord;
        std::stringstream ss(line);
        while (ss.good()) {
            std::string substr;
            std::getline(ss,substr, ',');
            float point = std::stof(substr);
            coord.push_back(point);
        }
        results.push_back(coord);
    }
    return results;
}

double norm2(std::vector<float> a, std::vector<float> b) {
    float r = 0;
    for (int i = 0; i < a.size(); i++) {
        r += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return std::sqrt(r);
}

Mat get_corres(const Mat& P, const Mat& M) {
    Mat Y(P.m_height, P.m_width);

    for (int i = 0; i < P.m_height; ++i)
    {
        std::vector<float> d;
        for (int k = 0; k < M.m_height; ++k)
            d.push_back(norm2(P[i], M[k]));
        auto min_idx = std::min_element(d.begin(), d.end()) - d.begin();
        for (int j = 0; j < M[min_idx].size(); ++j)
            Y[i][j] = M[min_idx][j];
    }

    return Y;
}

int main(int argc, char const *argv[])
{
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " path/to/test/file path/to/model/file";
        return 1;
    }
    std::ifstream file1(argv[1]);
    std::ifstream file2(argv[2]);
    if (!file1 || !file2) {
        std::cerr << "couldn't open file\n";
        return 1;
    }
    Mat test = parse_file(file1);
    Mat ref = parse_file(file2);

    //get_corres(test, ref).print();
    try {
        ref.print();
        test.print();
        ICP_matlab res(ref, test);
        res.get_p_transformed().print();
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
    }

    std::cout << "End\n";

    /*
    Mat first = {{{1,2,3}, {7,4,5}, {11,22,33}}};
    first.print();
    std::vector<float> v{1,2,3,4};
    Mat second(v);
    second.print();
    */
    return 0;
}
