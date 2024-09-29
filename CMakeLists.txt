cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(PROJECT_NAME "WHYLight")

if(DEFINED ENV{PROJECT_VER})
    # Strip any preceeding "v" from version (e.g. v1.1.1 > 1.1.1)
    message(INFO "Project version: $ENV{PROJECT_VER}")
    string(REGEX REPLACE "v" "" PROJECT_VER "$ENV{PROJECT_VER}")
else()
    message(INFO "Project version: unknown")
    set(PROJECT_VER "999.999.999")
endif()

project(${PROJECT_NAME})
