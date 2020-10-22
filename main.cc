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
    /*
    if (argc != 3) {
        std::cerr << "file must be given as argument\n";
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

    for (auto coord : get_correspondence_indices(test, ref)) {
        std::cout << std::get<0>(coord) << "," << std::get<1>(coord) << '\n';
    }
    */
    Mat first = {{{1,2,3}, {7,4,5}}};
    Mat second(first);
    Mat third = Mat::eye(3);
    first.print();
    second.print();
    third.print();
    second[1][1] = 10;
    first.print();
    second.print();
    return 0;
}
