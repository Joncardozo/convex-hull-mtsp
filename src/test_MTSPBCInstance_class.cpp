#include "MTSPBCInstance.hpp"
#include <cstddef>
#include <gtest/gtest.h>
#include <memory>
#include <string>


class MTSPBCInstanceTest : public ::testing::Test {

    protected:

    static std::unique_ptr<MTSPBCInstance> instance;

    static void SetUpTestSuite() {
        std::string filepath {"../experiments/BC/R1_5v_200n.bc"};
        std::string dist_filepath {"../experiments/inst.dat"};
        instance = std::make_unique<MTSPBCInstance>(filepath, dist_filepath);
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
