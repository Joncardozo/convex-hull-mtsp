#pragma once

#include <vector>

int read_instance(std::string file_name);           // read the instance from BCLIB
int read_inst_dist(std::string input_file);         // read distance matrix from pre processing
std::vector<int> find_hull();                       // find the hull for one vehicle
std::vector<std::vector<int>> find_onion_hull();    // find the hull for every vehicle
std::vector<int> find_route();                      // find the route for one vehicle
std::vector<std::vector<int>> find_solution();      // find heuristic solution
