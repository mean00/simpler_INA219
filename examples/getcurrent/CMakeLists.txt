#-----------------------------------------------------------------------------
#
# CMakeLists for lnDSO
# An opensource firmware for the DSO SHELL 150
# With optional hardware mods :
#   - Alternative rotary encoder wiring : MUCH Better, almost mandatory
#
#-----------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.0)
MESSAGE(STATUS "===============")
MESSAGE(STATUS "Starting ina219")
MESSAGE(STATUS "===============")
#____________________
# Options
#____________________

SET(LN_ENABLE_I2C  ON CACHE INTERNAL "")

SET(AF_FOLDER  ${CMAKE_SOURCE_DIR}/lnArduino/)
include(./mcuSelect.cmake)
SET(CMAKE_TOOLCHAIN_FILE ${AF_FOLDER}/lnArduino.cmake)

PROJECT(testIna219 NONE) # the NONE is very important !

enable_language(C CXX ASM) # this is important too!

#

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/include)
add_subdirectory(${AF_FOLDER})

include(${AF_FOLDER}/setup.cmake)

ADD_SUBDIRECTORY(externals/simpler_INA219)
include_directories(externals/simpler_INA219)

GENERATE_GD32_FIRMWARE(ina219 hello.cpp)

set_target_properties(ina219 PROPERTIES OUTPUT_NAME ina219${EXTENSION})
target_link_libraries(ina219 simpler_INA219)
MESSAGE(STATUS "\tFinal name is ${EXTENSION}")
