#pragma once


#include "MTSPBC.hpp"
#include "MTSPBC_util.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>


uint32_t add_convex_hull(MTSPBC& solution, const uint32_t vehicle, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord);
uint32_t find_onion_hull(MTSPBC& solution, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord);
uint32_t cheapest_insertion(MTSPBC& solution, std::vector<size_t>& un_nodes, const std::vector<Coord>& coord);
uint32_t remove_covered_nodes(MTSPBC& solution, std::vector<size_t>& un_nodes);
uint32_t assign_garage(MTSPBC& solution, std::vector<size_t>& un_nodes);
