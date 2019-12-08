cmake_minimum_required(VERSION 3.13.4)
project(test VERSION 1.0 LANGUAGES CXX)

#compilation c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Python3 COMPONENTS Development)

add_executable(test kmean.cpp)
target_link_libraries(test Python3::Python)
