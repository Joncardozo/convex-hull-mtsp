#pragma once

#include "Cht.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>


class ChtSol {
    private:
        uint32_t obj_;
        std::vector<Cht> tours_;
        bool feasible_;
        uint32_t max_distance_;
        int compute_obj_();

    public:
        ChtSol();
        uint32_t set_tour(const Cht& tour, const size_t vehicle);
        Cht get_tour(const size_t vehicle);
        uint32_t get_obj() const;
        bool is_feasible() const;
        uint32_t get_max_distance() const;
        std::vector<uint32_t> get_tour(size_t vehicle_index) const;
};
