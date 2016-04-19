cmake_minimum_required(VERSION 3.3)
project(Silicon)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES
    ece556.cpp
    ece556.h
    main.cpp
        svg.cpp svg.h astar.h)

set(FLUTE_OBJS
    obj/bookshelf_IO.o
    obj/dist.o
    obj/dl.o
    obj/err.o
    obj/flute.o
    obj/flute_mst.o
    obj/heap.o
    obj/memAlloc.o
    obj/mst2.o
    obj/neighbors.o
)

include_directories(include/)

add_executable(Silicon ${SOURCE_FILES} ${FLUTE_OBJS})
