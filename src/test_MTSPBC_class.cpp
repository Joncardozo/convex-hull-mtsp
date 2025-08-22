#include "MTSPBC.hpp"
#include "MTSPBC_util.hpp"
#include "chmtsp_util.hpp"
#include "MTSPBC_chh.hpp"
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>


class MTSPBCTest : public ::testing::Test {
    protected:

    static std::vector<std::vector<uint32_t>> read_matrix;
    static std::vector<size_t> un_nodes;
    static std::vector<Coord> coord;
    static uint32_t k_vehicles;
    static uint32_t n_nodes;
    static uint32_t r_radius;

    static void SetUpTestSuite() {
        std::string dist_file_path {"../experiments/inst.dat"};
        std::string cover_file_path {"../experiments/cover.dat"};
        std::string inst_file_path { "../experiments/BC/R1_5v_200n.bc" };
        read_instance(inst_file_path, coord, k_vehicles, n_nodes, r_radius);
        read_matrix = read_inst_dist(dist_file_path);
        read_cover(cover_file_path);
    }

    void SetUp() override {
        un_nodes.clear();
    }

};


std::vector<std::vector<uint32_t>> MTSPBCTest::read_matrix;
std::vector<size_t> MTSPBCTest::un_nodes;
std::vector<Coord> MTSPBCTest::coord;
uint32_t MTSPBCTest::k_vehicles;
uint32_t MTSPBCTest::n_nodes;
uint32_t MTSPBCTest::r_radius;


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
    ASSERT_GE(solution.get_obj_vehicle(1), 1);
    ASSERT_GE(solution.get_obj_vehicle(0), 1);
}


TEST_F(MTSPBCTest, ReadParams) {
    MTSPBC solution;
    for (uint32_t i { 0 }; i < coord.size(); i++) {
        un_nodes.push_back(i);
    }
    solution.create_distance_matrix(read_matrix);
    for (uint32_t i { 0 }; i < k_vehicles; i++) {
        solution.create_vehicle();
    }
    solution.set_radius(r_radius);
    ASSERT_EQ(solution.get_k_vehicles(), 5);
    ASSERT_EQ(solution.get_r_radius(), 10);
}


TEST_F(MTSPBCTest, CollectEvents) {
    MTSPBC solution;
    for (uint32_t i { 0 }; i < coord.size(); i++) {
        un_nodes.push_back(i);
    }
    solution.create_distance_matrix(read_matrix);
    for (uint32_t i { 0 }; i < k_vehicles; i++) {
        solution.create_vehicle();
    }
    solution.set_radius(r_radius);
    find_onion_hull(solution, un_nodes, coord);
    uint32_t max_obj { 0 };
    uint32_t n_events { 0 };
    for (uint32_t i { 0 }; i < k_vehicles; i++) {
        if (solution.get_obj_vehicle(i) > max_obj) {
            max_obj = solution.get_obj_vehicle(i);
        }
        n_events += solution.get_tour(i).size();
    }
    ASSERT_EQ(solution.get_events().back().first, max_obj);
    ASSERT_EQ(solution.get_events().size(), n_events);
}
