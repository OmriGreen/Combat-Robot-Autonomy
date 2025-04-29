cmake_minimum_required(VERSION 3.10)

project(Motion_Planning_CRA)

set(CMAKE_CXX_STANDARD 11)
find_package(ompl REQUIRED)

include_directories(${OMPL_INCLUDE_DIRS})

add_executable(simulation simulation.cpp dynamics.cpp CollisionChecking.cpp)
target_link_libraries(simulation ${OMPL_LIBRARIES})




