cmake_minimum_required(VERSION 3.16)

set(CCACHE_ENABLE OFF CACHE BOOL "Disable ccache for Unicode paths" FORCE)
set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(three-wheel-odometry-test)
