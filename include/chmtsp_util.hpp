#pragma once


#include <cstdint>
#include <vector>
#include <string>
#include "MTSPBC_util.hpp"


uint32_t read_instance(std::string file_name, std::vector<Coord>& coord, uint32_t& k_vehicles_p, uint32_t& n_nodes_p, uint32_t& r_radius_p);           // read the instance from BCLIB
std::vector<std::vector<uint32_t>> read_inst_dist(std::string input_file);         // read distance matrix from pre processing
uint32_t read_cover(std::string cover_file);        // read the cover pre processing data
std::vector<uint32_t> find_initial_hull();               // find the hull for one vehicle
std::vector<std::vector<uint32_t>> find_onion_hull();    // find the hull for every vehicle
std::vector<uint32_t> find_route();                      // find the route for one vehicle
std::vector<std::vector<uint32_t>> find_solution();      // find heuristic solution
std::vector<uint32_t> remove_covered_nodes(std::vector<uint32_t> hull);   // remove node from convex hull if the remaining hull covers it
void pruint32_t_hull(std::vector<uint32_t> hull);             // prints the hull order
uint32_t save_data(const std::vector<std::vector<uint32_t>>& onion_hull);        // save data for gnuplot
uint32_t save_data(const std::vector<uint32_t> &h, const std::string& hull_file_path);
uint32_t unassign(const std::vector<uint32_t>& hull);    // unassigns nodes from assigned hull nodes
std::vector<std::vector<uint32_t>> cheapest_insertion(std::vector<std::vector<uint32_t>> onion_hull);     // adds inner pouint32_ts to hulls
std::vector<std::vector<uint32_t>> assign_depot(std::vector<std::vector<uint32_t>> onion_hull);
uint32_t hull_objective(const std::vector<uint32_t>& hull);
bool check_feasibility(const std::vector<std::vector<uint32_t>>& onion_hull);
std::vector<std::vector<uint32_t>> fix_initial_route(std::vector<std::vector<uint32_t>> onion_hull);
