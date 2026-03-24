cmake_minimum_required(VERSION 4.0)
project(Roadef_challenge_2026)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(Roadef_challenge_2026 main.cpp)

target_include_directories(Roadef_challenge_2026 PRIVATE
        ${CMAKE_SOURCE_DIR}/networktools-main/networktools
)