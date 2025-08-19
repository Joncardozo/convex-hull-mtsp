// #include "chmtsp.hpp"
#include "chmtsp_util.cpp"
#include "chmtsp_util.hpp"
#include "Cht.hpp"
#include <iostream>

int main(int argc, char** argv) {

    std::string dist_file_path {"/Users/jonathan/code/mTSPBC/build/inst.dat"};
    std::string cover_file_path {"/Users/jonathan/code/mTSPBC/build/cover.dat"};
    std::string inst_file_path {argv[1]};

    read_instance(inst_file_path);
    read_inst_dist(dist_file_path);
    read_cover(cover_file_path);

    // encontra o hull inicial
    std::vector<int> hull_index { find_initial_hull() };
    std::cout << "hull inicial: " << std::endl;
    print_hull(hull_index);
    save_data(hull_index, "initial_hull.dat");

    // encontra um hull para cada veículo
    std::vector<std::vector<int>> onion_hull { find_onion_hull() };
    save_data(onion_hull);
    onion_hull = assign_depot(onion_hull);
    onion_hull = cheapest_insertion(onion_hull);
    onion_hull = fix_initial_route(onion_hull);
    // save_data(onion_hull);
    std::cout << "k-hull: " << std::endl;
    for (const auto& h : onion_hull) {
        print_hull(h);
    }

    // verifica se solucao é factível
    bool is_feasible { check_feasibility(onion_hull) };
    std::cout << "factibilidade: " << is_feasible << std::endl;

    // testa classe Cht (tour de um veículo)
    Cht tour_vehicle_0;


    return 0;
}
