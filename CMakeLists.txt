cmake_minimum_required(VERSION 3.0)

project(localmap)

add_executable(localmap main.cpp)

install(TARGETS localmap RUNTIME DESTINATION bin)
