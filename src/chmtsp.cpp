#include "chmtsp.hpp"
#include <cstddef>

// construtor
Cht::Cht(){
    obj_ = 0;
}

int Cht::insert_node(int node, size_t pos) {
    tour_.insert(tour_.begin() + pos, node);
    compute_obj();
    return obj_;
}

int Cht::get_obj() const {
    return obj_;
}

std::vector<int> Cht::get_tour() const {
    return tour_;
}
