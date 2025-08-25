#pragma once


#include "MTSPBC.hpp"
#include "MTSPBC_ds.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>


int orientation(const Coord& a, const Coord& b, const Coord& c);
Coord partial_coordinate(const Coord& m, const Coord& n, const double& dt);
double coord_norm(const Coord& coord);
uint32_t distance(const Nodes& a, const Nodes& b);
uint32_t distance(const Coord& a, const Coord& b);
uint32_t distance(const MTSPBC& solution, const uint32_t& event_index, const uint32_t& moving_vehicle);
// double coord_norm(const Coord& coord);
uint32_t unassign(const std::vector<uint32_t>& nodes, std::vector<size_t>& un_nodes);
