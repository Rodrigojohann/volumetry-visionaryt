#
# Copyright note: Redistribution and use in source, with or without modification, are permitted.
# 
# Created: August 2017
# 
# @author:	Andreas Richert
# SICK AG, Waldkirch
# email: TechSupport0905@sick.de
# 
# Last commit: $Date: 2017-09-27 10:44:05 +0200 (Mi, 27 Sep 2017) $
# Last editor: $Author: richean $
# 
# Version "$Revision: 14104 $"
#

project(sick_visionary_common)
cmake_minimum_required(VERSION 2.8)

include_directories(include)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++11")

### BUILD ###
aux_source_directory(. SRC_LIST)

add_library(${PROJECT_NAME} STATIC ${SRC_LIST})

if(WIN32)
  target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()
