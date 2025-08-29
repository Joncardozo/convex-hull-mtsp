#include "MTSPBC.hpp"
#include "MTSPBC_chh.hpp"
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <vector>


class MTSPBCTest : public ::testing::Test {
    protected:

    static std::vector<size_t> un_nodes;
    static std::unique_ptr<MTSPBCInstance> instance;

    static void SetUpTestSuite() {
        std::string dist_filepath {"../experiments/inst.dat"};
        std::string filepath { "../experiments/BC/R1_5v_200n.bc" };
        instance = std::make_unique<MTSPBCInstance>(filepath, dist_filepath);
    }

    void SetUp() override {
        un_nodes.clear();
    }

    static void TearDownTestSuite() {
        instance.reset();
    }

};


std::vector<size_t> MTSPBCTest::un_nodes;
std::unique_ptr<MTSPBCInstance> MTSPBCTest::instance = nullptr;


TEST_F(MTSPBCTest, CreateOneRoute) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);

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
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);

    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    solution.create_vehicle();
    solution.create_vehicle();
    // add_convex_hull(solution, 1, un_nodes, coord);
    find_onion_hull(solution, un_nodes, cref);
    int x { 0 };
    ASSERT_GE(solution.get_obj_vehicle(1), 1);
    ASSERT_GE(solution.get_obj_vehicle(0), 1);
}


TEST_F(MTSPBCTest, ReadParams) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    ASSERT_EQ(solution.get_k_vehicles(), 5);
    ASSERT_EQ(solution.get_r_radius(), 10);
}


TEST_F(MTSPBCTest, CollectEvents) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    find_onion_hull(solution, un_nodes, cref);
    uint32_t max_obj { 0 };
    uint32_t n_events { 0 };
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        if (solution.get_obj_vehicle(i) > max_obj) {
            max_obj = solution.get_obj_vehicle(i);
        }
        n_events += solution.get_tour(i).size();
    }
    int debug { 0 };
    ASSERT_EQ(solution.get_events().back().first, max_obj);
    ASSERT_EQ(solution.get_events().size(), n_events);
}


TEST_F(MTSPBCTest, CheapestInsertion) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    find_onion_hull(solution, un_nodes, cref);
    ASSERT_NO_THROW(cheapest_insertion(solution, un_nodes, cref));
    int debug { 0 };
    ASSERT_TRUE(un_nodes.size() == 0);
    ASSERT_EQ(solution.get_total_obj(), 2396);
}


TEST_F(MTSPBCTest, AssignDepot) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    find_onion_hull(solution, un_nodes, cref);
    ASSERT_NO_THROW(cheapest_insertion(solution, un_nodes, cref));
    assign_garage(solution, un_nodes);
    int debug { 0 };
    ASSERT_TRUE(un_nodes.size() == 0);
    ASSERT_GE(solution.get_total_obj(), 2397);
    for (uint32_t i { 0 }; i < solution.get_k_vehicles(); i++) {
        int debug {};
        ASSERT_TRUE(solution.get_pos_for_node(i, 0));
    }
}


TEST_F(MTSPBCTest, CloseTours) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    find_onion_hull(solution, un_nodes, cref);
    ASSERT_NO_THROW(cheapest_insertion(solution, un_nodes, cref));
    assign_garage(solution, un_nodes);
    ASSERT_TRUE(un_nodes.size() == 0);
    ASSERT_GE(solution.get_total_obj(), 2397);
    close_tours(solution);
    for (uint32_t i { 0 }; i < solution.get_k_vehicles(); i++) {
        int debug {};
        ASSERT_TRUE(solution.get_pos_for_node(i, 0));
        ASSERT_TRUE(solution.get_complete_tour(i));
    }
    int debug {};
}


TEST_F(MTSPBCTest, SaveSolution) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    find_onion_hull(solution, un_nodes, cref);
    ASSERT_NO_THROW(cheapest_insertion(solution, un_nodes, cref));
    assign_garage(solution, un_nodes);
    ASSERT_TRUE(un_nodes.size() == 0);
    ASSERT_GE(solution.get_total_obj(), 2397);
    close_tours(solution);
    for (uint32_t i { 0 }; i < solution.get_k_vehicles(); i++) {
        int debug {};
        ASSERT_TRUE(solution.get_pos_for_node(i, 0));
        ASSERT_TRUE(solution.get_complete_tour(i));
    }
    solution.save_solution("../data/points.dat", "../data/tour.dat");
    std::ifstream points_file("../data/points.dat");
    std::ifstream tour_file("../data/tour.dat");
    ASSERT_TRUE(points_file.is_open());
    ASSERT_TRUE(tour_file.is_open());
    points_file.close();
    tour_file.close();
    int debug {};
}
