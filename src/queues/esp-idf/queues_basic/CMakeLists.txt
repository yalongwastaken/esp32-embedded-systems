# minimum CMake version
cmake_minimum_required(VERSION 3.16)

# include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# project name
project(queues_basic)