cmake_minimum_required(VERSION 3.2.0)
project(MeshSeg)

set(CMAKE_CXX_STANDARD 17)

add_executable(MeshSeg 
    src/main.cpp 
    src/mesh.h 
    src/mesh.cpp 
    src/vector3.h 
    src/flow.h
    src/flow.cpp)