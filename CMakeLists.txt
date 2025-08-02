
add_library(chmtsp_lib src/chmtsp.cpp)
add_library(chmtsp_util_lib src/chmtsp_util.cpp)

target_include_directories(chmtsp_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(chmtsp_util_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(test_chmtsp_lib src/test_main.cpp)

target_link_libraries(test_chmtsp_lib PRIVATE chmtsp_lib chmtsp_util_lib)
