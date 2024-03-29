cmake_minimum_required(VERSION 2.8)

project(iCubParis02_data_analysis)

#### Find kdl_codyco, require that kdl_codyco version is at least kdl_codyco_REQVERSION
set(kdl_codyco_REQVERSION_MAJOR "0")
set(kdl_codyco_REQVERSION_MINOR "0")
set(kdl_codyco_REQVERSION_PATCH "2")
set(kdl_codyco_REQVERSION_TWEAK "")
if(kdl_codyco_REQVERSION_TWEAK)
    set(kdl_codyco_REQVERSION ${kdl_codyco_REQVERSION_MAJOR}.${kdl_codyco_REQVERSION_MINOR}.${kdl_codyco_REQVERSION_PATCH}.${kdl_codyco_REQVERSION_TWEAK})
else()
    set(kdl_codyco_REQVERSION ${kdl_codyco_REQVERSION_MAJOR}.${kdl_codyco_REQVERSION_MINOR}.${kdl_codyco_REQVERSION_PATCH})
endif()

find_package(kdl_codyco REQUIRED)

message(STATUS "kdl_codyco is version: ${kdl_codyco_VERSION}")

if(kdl_codyco_VERSION VERSION_LESS kdl_codyco_REQVERSION)
    message(FATAL_ERROR "kdl_codyco version is not compatible, please update kdl_codyco")
else()
    message(STATUS "found compatible kdl_codyco version")
endif()

find_package(kdl_format_io REQUIRED)

include_directories(${orocos_kdl_INCLUDE_DIRS} ${kdl_codyco_INCLUDE_DIRS} ${kdl_format_io_INCLUDE_DIRS})

add_executable(iCubParis02_data_analysis iCubParis02_data_analysis.cpp)

target_link_libraries(iCubParis02_data_analysis ${orocos_kdl_LIBRARIES} kdl_codyco kdl_format_io)

