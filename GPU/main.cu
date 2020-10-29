// #include <fstream>
// #include <string>
// #include <cstring>
// #include <sstream>
// #include <iostream>
#include "matrices.cuh"
// #include "icp.hh"
// #include "icp-matlab.hh"

// Mat parse_file(std::ifstream &file) {
//     std::string line;
//     int counter = 0;
//     float** results;
//     // std::vector<std::vector<float> > results;
//     while (std::getline(file, line))
//     {
//         if (!counter) {
//             counter++;
//             continue;
//         }
//         float* coord;
//         std::vector<float> coord;
//         std::stringstream ss(line);
//         while (ss.good()) {
//             std::string substr;
//             std::getline(ss,substr, ',');
//             float point = std::stof(substr);
//             coord.push_back(point);
//         }
//         results.push_back(coord);
//     }
//     return results;
// }

int main(int argc, char const *argv[])
{
    Mat A = Mat(10, 8, 3);
    A.print();

    Mat G = Mat(10, 8);
    G.print();

    float *v = (float*)calloc(4, sizeof(float));
    v[3] = 4;
    Mat B = Mat(v, 2, 2);
    B.print();
    Mat C = Mat(B);
    C.m_buffer[2] = 2;
    B.print();
    C.print();
    B.dot(C).print();
    v[0] = 5;
    free(v);

    Mat K = Mat(2,3,1);
    for (int i = 0; i < 6; ++i)
        K.m_buffer[i] = i;
    Mat L = Mat(3,4,2);
    for (int i = 0; i < 12; ++i)
        L.m_buffer[i] = i + 6;
    K.print();
    L.print();
    K.dot(L).print();

    Mat M(3, 8);
    for (int i = 0; i < 24; ++i)
        M.m_buffer[i] = i;
    M.print();
    M.T().print();

    (K + K).print();
    Mat O(1, 3);
    for (int i = 0; i < 3; ++i)
        O.m_buffer[i] = i * 3;
    (K + O).print();

    Mat P(2,2);
    P.m_buffer[0] = 5;
    P.m_buffer[1] = 2;
    P.m_buffer[2] = -7;
    P.m_buffer[3] = -3;
    P.print();
    P.inverse().print();

    // if (argc != 3) {
    //     std::cerr << "Usage: " << argv[0] << " path/to/test/file path/to/model/file";
    //     return 1;
    // }
    // std::ifstream file1(argv[1]);
    // std::ifstream file2(argv[2]);
    // if (!file1 || !file2) {
    //     std::cerr << "couldn't open file\n";
    //     return 1;
    // }
    // Mat test = parse_file(file1);
    // Mat ref = parse_file(file2);

    //try {
    //    ref.print();
    //    test.print();
    //    ICP_matlab res(ref, test);
    //    res.get_p_transformed().print();
    //} catch (const char* msg) {
    //    std::cerr << msg << std::endl;
    //}

    // try {
    //     ref.print();
    //     test.print();
    //     icp icp(3);
    //     auto res = icp.icp_least_squares(test, ref);
    //     res.print();
    // } catch (const char* msg) {
    //     std::cerr << msg << std::endl;
    // }

    // std::cout << "End\n";

    /*
    Mat first = {{{1,2,3}, {7,4,5}, {11,22,33}}};
    first.print();
    std::vector<float> v{1,2,3,4};
    Mat second(v);
    second.print();
    */
    return 0;
}
