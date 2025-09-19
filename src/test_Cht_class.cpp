#include "Cht.hpp"
#include "MTSPBCInstance.hpp"
#include <cstddef>
#include <cstdint>
#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>
#include <vector>


// setup inicial - ler instância
class ChmtspTest : public ::testing::Test {

    protected:

    static std::unique_ptr<MTSPBCInstance> instance;

    static void SetUpTestSuite() {
        std::string filepath {"../experiments/BC/R1_5v_200n.bc"};
        std::string dist_filepath {"../experiments/inst.dat"};
        std::string cover_filepath { "../experiments/cover.dat" };
        instance = std::make_unique<MTSPBCInstance>(filepath, dist_filepath, cover_filepath);
    }

    static void TearDownTestSuite() {
        instance.reset();
    }

};


std::unique_ptr<MTSPBCInstance> ChmtspTest::instance = nullptr;



// testa se o valor da funcão objetivo é calculada corretamente
TEST_F(ChmtspTest, ComputesObjClass) {
    // testa classe Cht (tour de um veículo)
    Cht tour_vehicle_0;
    const MTSPBCInstance& cref = *instance;
    tour_vehicle_0.push_front(1, cref);
    tour_vehicle_0.push_front(3, cref);
    tour_vehicle_0.push_front(0, cref);
    tour_vehicle_0.push_back(0, cref);

    uint32_t obj { tour_vehicle_0.get_obj() };

    ASSERT_EQ(obj, 194);
}


// testa todas as inserções e remoções de nós
TEST_F(ChmtspTest, InsertionDeletion) {
    // instancia objeto da classe
    Cht tour_vehicle_0;
    const MTSPBCInstance& cref = *instance;
    tour_vehicle_0.push_front(2, cref);
    tour_vehicle_0.push_front(5, cref);
    tour_vehicle_0.insert_node(1, 1, cref);
    tour_vehicle_0.remove_node(2, cref);
    tour_vehicle_0.insert_node(3, 1, cref);
    tour_vehicle_0.insert_node(4, 0, cref);
    tour_vehicle_0.remove_node(3, cref);
    tour_vehicle_0.push_back(2, cref);
    tour_vehicle_0.remove_node(0, cref);

    bool complete_tour{ tour_vehicle_0.check_complete_tour_() };

    std::vector<uint32_t> expected_route {5, 3, 2};
    std::vector<uint32_t> route { tour_vehicle_0.get_tour() };

    EXPECT_EQ(expected_route, route);
    EXPECT_EQ(complete_tour, false);
}


// testa exceções na inserção/remoção de nós fora de limite de índice
TEST_F(ChmtspTest, ThrowExceptionBounds) {
    Cht tour_vehicle_0;
    const MTSPBCInstance& cref = *instance;

    ASSERT_THROW(tour_vehicle_0.pop_back(cref), std::logic_error);
    ASSERT_THROW(tour_vehicle_0.pop_front(cref), std::logic_error);
    ASSERT_THROW(tour_vehicle_0.remove_node(1, cref), std::logic_error);
}


// testa se a lista de eventos é calculada corretamente
TEST_F(ChmtspTest, GetEventLists) {
    Cht tour_test;
    const MTSPBCInstance& cref = *instance;
    tour_test.push_back(5, cref);
    tour_test.push_back(3, cref);
    tour_test.push_back(2, cref);
    std::vector<uint32_t> expected_events { 0, 71,  150 };
    std::vector<uint32_t> events { tour_test.get_events() };
    EXPECT_EQ(events, expected_events);
}

TEST_F(ChmtspTest, IsCompleteRoute) {
    Cht tour_test;
    const MTSPBCInstance& cref = *instance;

    tour_test.push_back(5, cref);
    tour_test.push_back(3, cref);
    tour_test.push_back(2, cref);
    tour_test.push_back(0, cref);
    tour_test.push_front(0, cref);
    bool complete_route { tour_test.get_complete() };
}


// testa se é possível obter a lista de nós da rota
TEST_F(ChmtspTest, GetRoute) {
    Cht tour_test;
    const MTSPBCInstance& cref = *instance;

    tour_test.push_back(5, cref);
    tour_test.push_back(3, cref);
    tour_test.push_back(2, cref);
    tour_test.push_back(0, cref);
    tour_test.push_front(0, cref);
    std::vector<uint32_t> got_route { tour_test.get_tour() };
    std::vector<uint32_t> expected_route { 0, 5, 3, 2, 0 };
    EXPECT_EQ(got_route, expected_route);
}


// testa se é possível receber um nó a partir de um índice da rota
TEST_F(ChmtspTest, GetNodePos) {
    Cht tour_test;
    const MTSPBCInstance& cref = *instance;

    tour_test.push_back(5, cref);
    tour_test.push_back(3, cref);
    tour_test.push_back(2, cref);
    tour_test.push_back(0, cref);
    tour_test.push_front(0, cref);
    uint32_t node { tour_test.get_node_at_pos(2) };
    EXPECT_EQ(node, 3);
}


// testa se recebe um índice a partir de um nó, se houver o nó na rota
TEST_F(ChmtspTest, GetPosNode) {
    Cht tour_test;
    const MTSPBCInstance& cref = *instance;

    tour_test.push_back(5, cref);
    tour_test.push_back(3, cref);
    tour_test.push_back(2, cref);
    tour_test.push_back(0, cref);
    tour_test.push_front(0, cref);
    auto index { tour_test.get_pos_for_node(3) };
    EXPECT_EQ(index, 2);
}
