#pragma once

#include <cstddef>
#include <vector>

class Cht {
    private:
        int obj_;
        int n_nodes_;
        std::vector<int> tour_;
        int compute_obj();

    public:
        Cht();
        int insert_node(int node, size_t pos);
        int get_obj() const;
        std::vector<int> get_tour() const;
};


int testa_build();
