if(SOLO)
    cmake_minimum_required(VERSION 3.15)
    set(CMAKE_CXX_STANDARD 20)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    project(mTSPBC CXX)

    enable_testing()

    include(FetchContent)
    FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.17.0
)
    FetchContent_MakeAvailable(googletest)

endif()

add_library(Cht_lib src/Cht.cpp)
add_library(MTSPBC_lib src/MTSPBC.cpp)
add_library(MTSPBC_chh_lib src/MTSPBC_chh.cpp src/MTSPBC_util.cpp)
add_library(MTSPBCInstance_lib src/MTSPBCInstance.cpp)

target_include_directories(Cht_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(MTSPBC_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(MTSPBC_chh_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(MTSPBCInstance_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(BUILD_TESTING)
    add_executable(test_Cht_class src/test_Cht_class.cpp)
    add_executable(test_MTSPBC_class src/test_MTSPBC_class.cpp)
    add_executable(test_MTSPBCInstance_class src/test_MTSPBCInstance_class.cpp)
    target_link_libraries(test_Cht_class PRIVATE Cht_lib MTSPBCInstance_lib MTSPBC_chh_lib GTest::gtest_main)
    target_link_libraries(test_MTSPBC_class PRIVATE MTSPBCInstance_lib MTSPBC_lib MTSPBC_chh_lib Cht_lib GTest::gtest_main)
    target_link_libraries(test_MTSPBCInstance_class PRIVATE MTSPBC_lib Cht_lib MTSPBCInstance_lib GTest::gtest_main)
    include(GoogleTest)
    gtest_discover_tests(test_Cht_class)
    gtest_discover_tests(test_MTSPBC_class)
    gtest_discover_tests(test_MTSPBCInstance_class)
endif()
