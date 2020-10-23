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

    //try {
    //    ref.print();
    //    test.print();
    //    ICP_matlab res(ref, test);
    //    res.get_p_transformed().print();
    //} catch (const char* msg) {
    //    std::cerr << msg << std::endl;
    //}

    try {
        ref.print();
        test.print();
        icp icp(3);
        auto res = icp.icp_least_squares(test, ref);
        res.print();
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
