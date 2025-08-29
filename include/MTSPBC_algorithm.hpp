#pragma once


#include "MTSPBC.hpp"
#include <vector>
#include <algorithm>


namespace MTSPBC_Algorithm {

    void opt_2();
    void opt_3();
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

}
