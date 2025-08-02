#include "chmtsp.hpp"
#include "chmtsp_util.cpp"
#include "chmtsp_util.hpp"
#include <iostream>

int main(int argc, char** argv) {

    std::string dist_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/bin/inst.dat"};
    std::string inst_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/instances/BC/toyzinha.bc"};

    read_instance(inst_file_path);
    read_inst_dist(dist_file_path);

    std::vector<int> hull_index { find_initial_hull() };
    for (const auto& i : hull_index) {
        std::cout << i << " " << std::endl;
    }

    return 0;
}
