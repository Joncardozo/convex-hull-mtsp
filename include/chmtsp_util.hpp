#pragma once

#include <vector>

int read_instance(std::string file_name);           // read the instance from BCLIB
int read_inst_dist(std::string input_file);         // read distance matrix from pre processing
int read_cover(std::string cover_file);        // read the cover pre processing data
std::vector<int> find_initial_hull();               // find the hull for one vehicle
std::vector<std::vector<int>> find_onion_hull();    // find the hull for every vehicle
std::vector<int> find_route();                      // find the route for one vehicle
std::vector<std::vector<int>> find_solution();      // find heuristic solution
std::vector<int> remove_covered_nodes(std::vector<int> hull);   // remove node from convex hull if the remaining hull covers it
void print_hull(std::vector<int> hull);             // prints the hull order
int save_data(const std::vector<std::vector<int>>& onion_hull);        // save data for gnuplot
int save_data(const std::vector<int> &h, const std::string& hull_file_path);
int unassign(const std::vector<int>& hull);    // unassigns nodes from assigned hull nodes
std::vector<std::vector<int>> cheapest_insertion(std::vector<std::vector<int>> onion_hull);     // adds inner points to hulls
std::vector<std::vector<int>> assign_depot(std::vector<std::vector<int>> onion_hull);
int hull_objective(const std::vector<int>& hull);
bool check_feasibility(const std::vector<std::vector<int>>& onion_hull);
std::vector<std::vector<int>> fix_initial_route(std::vector<std::vector<int>> onion_hull);
