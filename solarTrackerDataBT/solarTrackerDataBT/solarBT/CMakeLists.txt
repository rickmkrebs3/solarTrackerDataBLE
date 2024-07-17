cmake_minimum_required(VERSION 3.28)
project(solarBT)

set(CMAKE_CXX_STANDARD 17)

add_executable(solarBT main.cpp
        src/solarBTAdapter.h
        src/solarBTAdapter.cpp)
