#pragma once

#include <vector>

std::vector<uint32_t> find_initial_hull();               // find the hull for one vehicle
std::vector<std::vector<uint32_t>> find_onion_hull();    // find the hull for every vehicle
std::vector<uint32_t> find_route();                      // find the route for one vehicle
std::vector<std::vector<uint32_t>> find_solution();      // find heuristic solution
std::vector<uint32_t> remove_covered_nodes(std::vector<uint32_t> hull);   // remove node from convex hull if the remaining hull covers it
