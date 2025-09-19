#include "MTSPBCInstance.hpp"
#include <cstddef>
#include <gtest/gtest.h>
#include <memory>
#include <string>


class MTSPBCInstanceTest : public ::testing::Test {

    protected:

    static std::unique_ptr<MTSPBCInstance> instance;

    static void SetUpTestSuite() {
        std::string filepath { "../experiments/BC/R1_5v_200n.bc" };
        std::string dist_filepath { "../experiments/inst.dat" };
        std::string cover_filepath { "../experiments/cover.dat" };
        instance = std::make_unique<MTSPBCInstance>(filepath, dist_filepath, cover_filepath);
    }

    static void TearDownTestSuite() {
        instance.reset();
    }

};

std::unique_ptr<MTSPBCInstance> MTSPBCInstanceTest::instance = nullptr;


TEST_F(MTSPBCInstanceTest, CreateInstance) {
    ASSERT_NE(instance, nullptr);
    EXPECT_EQ(instance->n(), 199);
    EXPECT_EQ(instance->k(), 5);
    EXPECT_EQ(instance->r(), 10);
}

TEST_F(MTSPBCInstanceTest, GotLBUB) {
    EXPECT_NEAR(instance->get_LB(198, 197, 198), 39.79591836734694, 1e-3);
    EXPECT_NEAR(instance->get_UB(198, 197, 198), 50, 1e-3);
}
