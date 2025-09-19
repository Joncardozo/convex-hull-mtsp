#include "MTSPBC.hpp"
#include "MTSPBCInstance.hpp"
#include "MTSPBC_chh.hpp"
#include "MTSPBC_util.hpp"
#include "MTSPBC_algorithm.hpp"
#include <gtest/gtest.h>
#include <memory>
#include <fstream>
#include <vector>


class LocalSearchTest : public ::testing::Test {
    protected:

    static std::vector<size_t> un_nodes;
    static std::unique_ptr<MTSPBCInstance> instance;

    static void SetUpTestSuite() {
        std::string dist_filepath {"../experiments/inst.dat"};
        std::string filepath { "../experiments/BC/R1_5v_200n.bc" };
        std::string cover_filepath { "../experiments/cover.dat" };
        instance = std::make_unique<MTSPBCInstance>(filepath, dist_filepath, cover_filepath);
    }

    void SetUp() override {
        un_nodes.clear();
    }

    static void TearDownTestSuite() {
        instance.reset();
    }

};


std::vector<size_t> LocalSearchTest::un_nodes;
std::unique_ptr<MTSPBCInstance> LocalSearchTest::instance = nullptr;


TEST_F(LocalSearchTest, MinMaxDist3Opt) {
    const MTSPBCInstance& cref = *instance;
    MTSPBC solution(cref);
    for (uint32_t i { 0 }; i < cref.n(); i++) {
        un_nodes.push_back(i);
    }
    for (uint32_t i { 0 }; i < cref.k(); i++) {
        solution.create_vehicle();
    }
    solution.set_radius(cref.r());
    for (auto i { 0 }; i < cref.k(); i++) {
        add_convex_hull(solution, i, un_nodes, cref);
        unassign(solution.get_tour(i), un_nodes);
        remove_covered_nodes(solution, cref, i, un_nodes);

    }
    assign_garage(solution, un_nodes);
    close_tours(solution);

    ASSERT_NO_THROW(cheapest_insertion(solution, un_nodes, cref, true));
    ASSERT_NO_THROW(minimize_e_dist(solution, cref));

    solution.save_solution("../data/points.dat", "../data/tour.dat");
    std::ifstream points_file("../data/points.dat");
    std::ifstream tour_file("../data/tour.dat");
    ASSERT_TRUE(points_file.is_open());
    ASSERT_TRUE(tour_file.is_open());
    points_file.close();
    tour_file.close();
    int debug {};
}
