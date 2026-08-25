cmake_minimum_required(VERSION 3.16)

# ccache 4.12.1 on Windows cannot process this workspace's Chinese path.
set(CCACHE_ENABLE OFF CACHE BOOL "Disable ccache for Unicode paths" FORCE)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(car-hardware-test)
