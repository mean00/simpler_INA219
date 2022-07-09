
SET(SRCS simpler_INA219.cpp )
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(simpler_INA219 STATIC ${SRCS})
target_include_directories(simpler_INA219 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
