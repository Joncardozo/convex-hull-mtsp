#pragma once


#include "MTSPBC.hpp"
#include "MTSPBC_ds.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>


int orientation(const Coord& a, const Coord& b, const Coord& c);
Coord partial_coordinate(const Coord& m, const Coord& n, const double& dt);
double coord_norm(const Coord& coord);
Coord real_position(const MTSPBC& solution, const uint32_t moving_vehicle, const uint32_t last_e_mv, const uint32_t last_e_mv_i, const uint32_t e_time);
uint32_t distance(const Nodes& a, const Nodes& b);
uint32_t distance(const Coord& a, const Coord& b);
uint32_t distance(const MTSPBC& solution, const uint32_t event_index, const uint32_t moving_vehicle);
uint32_t distance(const MTSPBC& solution, const uint32_t event_index, const uint32_t moving_vehicle_1, const uint32_t moving_vehicle_2);
// double coord_norm(const Coord& coord);
uint32_t unassign(const std::vector<uint32_t>& nodes, std::vector<size_t>& un_nodes);
