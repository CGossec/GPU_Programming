#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include "matrices.hh"
#include "icp.hh"

std::vector<std::array<float, 3> > parse_file(std::ifstream &file) {
    std::string line;
    int counter = 0;
    std::vector<std::array<float, 3> > results;
    while (std::getline(file, line))
    {
        if (!counter) {
            counter++;
            continue;
        }
        std::array<float, 3> coord;
        std::stringstream ss(line);
        int pos = 0;
        while (ss.good()) {
            std::string substr;
            std::getline(ss,substr, ',');
            float point = std::stof(substr);
            coord.at(pos++) = point;
        }
        results.push_back(coord);
    }
    return results;
}

int main(int argc, char const *argv[])
{
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
    std::vector<std::array<float, 3> > test = parse_file(file1);
    std::vector<std::array<float, 3> > ref = parse_file(file2);

    for (auto coord : get_correspondence_indices(test, ref)) {
        std::cout << std::get<0>(coord) << "," << std::get<1>(coord) << '\n';
    }

    return 0;
}