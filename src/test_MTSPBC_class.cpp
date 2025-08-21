#include "MTSPBC.hpp"
#include "chmtsp_util.hpp"
#include <cstdint>
#include <gtest/gtest.h>
#include <vector>


class MTSPBCTest : public ::testing::Test {
    protected:
    static void SetUpTestSuite() {
        std::string dist_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/bin/inst.dat"};
        std::string cover_file_path {"/Users/jonathan/code/mTSPBC/BackCovered/bin/cover.dat"};
        std::string inst_file_path { "/Users/jonathan/code/mTSPBC/BackCovered/instances/BC/R1_5v_200n.bc" };
        read_instance(inst_file_path);
        read_matrix = read_inst_dist(dist_file_path);
        read_cover(cover_file_path);
    }

    public:
    static std::vector<std::vector<uint32_t>> read_matrix;
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
    std::vector<uint32_t> got_route { solution.get_complete_tour(0) };
    std::vector<uint32_t> expected_route { 0, 5, 3, 2, 2 };
    ASSERT_EQ(expected_route, got_route);
    uint32_t k_vehicles { solution.get_k_vehicles() };
    ASSERT_EQ(k_vehicles, 1);
    uint32_t obj { solution.get_total_obj() };
    ASSERT_EQ(obj, 205);
}
