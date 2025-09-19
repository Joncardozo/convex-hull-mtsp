#pragma once


#include "MTSPBC.hpp"
#include "MTSPBCInstance.hpp"


void opt_2();
void opt_3(MTSPBC& solution, const MTSPBCInstance& instance, const uint32_t k_1, const uint32_t k_2, const uint32_t tour_1_open_edge_1, const uint32_t tour_1_open_edge_2, const uint32_t tour_2_open_edge_1);
void opt_4();
void opt_5();
void swap();
void reverse();

// intra route moves
void or_opt();
void node_relocation();
void geni();

// inter route moves
void node_relocation();
void node_swap();
void seg_exchange();

// ready to use local search
void minimize_e_dist(MTSPBC& solution, const MTSPBCInstance& instance);
void minimize_e_dist_2(MTSPBC& solution, const MTSPBCInstance& instance);
