#include <fstream>
#include <string>
#include <cstring>
#include <sstream>
#include <iostream>
#include "matrices.cuh"
#include "icp.cuh"

Mat parse_file(std::ifstream &file) {
    std::string line;
    float* results = NULL;
    int counter = 0;
    int index = 0;
    int res_size = 0;
    int height = 0;
    while (std::getline(file, line))
    {
        if (!counter) {
            counter++;
            continue;
        }
        std::stringstream ss(line);
        res_size += 1;
        results = (float*)realloc(results, 3 * res_size * sizeof(float));
        int cpt = 0;
        while (ss.good()) {
            if (cpt >= 3)
                break;
            std::string substr;
            std::getline(ss,substr, ',');
            float point = std::stof(substr);
            results[index++] = point;
            cpt++;
        }
        if (cpt != 3) {
            std::cerr << "ICP only accept 3D points\n";
            throw "ICP only accept 3D points\n";
        }
        height++;
    }
    Mat res = Mat(results, height, 3);
    return res;
}

int main(int argc, char const *argv[])
{
    int nb_iterations = 30;
    bool force_iteration = false;

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << "[-it nb_iteration] path/to/test/file path/to/model/file";
        return 1;
    }
    else if (argc == 5)
    {
        force_iteration = true;
        nb_iterations = std::stoi(argv[2]);
    }

    std::ifstream file1(argv[1 + 2 * force_iteration]);
    std::ifstream file2(argv[2 + 2 * force_iteration]);
    if (!file1 || !file2) {
        std::cerr << "couldn't open file\n";
        return 1;
    }
    Mat test = parse_file(file1);
    Mat ref = parse_file(file2);

    try {
        //ref.print();
        //test.print();
        icp res = icp(test, ref);
        res.fit(nb_iterations, 0.0001, force_iteration);
        //res.get_src_transformed().print();
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
    }

    std::cout << "End\n";
    return 0;
}
