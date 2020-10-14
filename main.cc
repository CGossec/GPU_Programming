#include <vector>
#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>

std::vector<std::vector<float> > parse_file(std::ifstream &file) {
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
    if (argc != 2) {
        std::cerr << "file must be given as argument\n";
        return 1;
    }
    std::ifstream file(argv[1]);
    if (!file) {
        std::cerr << "couldn't open file\n";
        return 1;
    }
    std::vector<std::vector<float> > results = parse_file(file);
    for (auto coord : results) {
        for (auto point : coord) {
            std::cout << point << ",";
        }
        std::cout << '\n';
    }
    return 0;
}
