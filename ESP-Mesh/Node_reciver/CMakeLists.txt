# The following four lines of boilerplate have to be in your project's CMakeLists From Reviver 
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)   

set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/esp-idf-lib/components)
#set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../components)


include($ENV{MDF_PATH}/project.cmake)
#include($ENV{IDF_PATH}/tools/cmake/project.cmake)


project(Node_reciver)
