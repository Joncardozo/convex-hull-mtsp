#include "MTSPBC.hpp"
#include "MTSPBC_util.hpp"
#include "chmtsp_util.hpp"
#include "MTSPBC_chh.hpp"
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>


std::vector<std::vector<uint32_t>> read_matrix;
std::vector<size_t> un_nodes;
std::vector<Coord> coord;


class MTSPBCTest : public ::testing::Test {
    protected:
    static void SetUpTestSuite() {
        std::string dist_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/bin/inst.dat"};
        std::string cover_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/bin/cover.dat"};
        std::string inst_file_path { "/Users/jonathan/code/mTSPBC/BackCovered/instances/BC/R1_5v_200n.bc" };
        read_instance(inst_file_path, coord);
        read_matrix = read_inst_dist(dist_file_path);
        read_cover(cover_file_path);
    }
};


TEST_F(MTSPBCTest, CreateOneRoute) {
    MTSPBC solution;
    solution.create_distance_matrix(read_matrix);
    solution.create_vehicle();
    solution.push_back(0, 0);
    solution.push_back(0, 5);
    solution.push_back(0, 3);
    solution.push_back(0, 2);
    solution.push_back(0, 0);
    std::vector<uint32_t> got_route { solution.get_tour(0) };
    std::vector<uint32_t> expected_route { 0, 5, 3, 2, 0 };
    ASSERT_EQ(expected_route, got_route);
    uint32_t k_vehicles { solution.get_k_vehicles() };
    ASSERT_EQ(k_vehicles, 1);
    uint32_t obj { solution.get_total_obj() };
    ASSERT_EQ(obj, 205);
}


TEST_F(MTSPBCTest, FindInitialHull) {
    MTSPBC solution;
    for (uint32_t i { 0 }; i < coord.size(); i++) {
        un_nodes.push_back(i);
    }
    solution.create_vehicle();
    solution.create_vehicle();
    solution.create_distance_matrix(read_matrix);
    // add_convex_hull(solution, 1, un_nodes, coord);
    find_onion_hull(solution, un_nodes, coord);
    int x { 0 };
    ASSERT_GE(solution.get_total_obj(), 1);
}
