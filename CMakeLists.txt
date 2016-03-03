cmake_minimum_required(VERSION 3.3)
project(Silicon)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(BOOST_LIBS
    boost_system
    boost_filesystem
)

set(SOURCE_FILES
    ece556.cpp
    ece556.h
    main.cpp)

link_directories(lib/)
include_directories(include/)

add_custom_command(
        OUTPUT include/boost
        COMMAND make include/boost
        COMMENT "Building boost"
        VERBATIM
)

add_executable(Silicon ${SOURCE_FILES})
target_link_libraries(Silicon ${BOOST_LIBS})
