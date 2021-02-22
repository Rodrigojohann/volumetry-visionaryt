#
# Copyright note: Redistribution and use in source, with or without modification, are permitted.
# 
# Created: November 2016
# 
# @author:	Andreas Richert
# SICK AG, Waldkirch
# email: TechSupport0905@sick.de
# 
# Last commit: $Date: 2019-07-05 07:55:11 +0200 (Fr, 05 Jul 2019) $
# Last editor: $Author: peichto $
# 
# Version "$Revision: 22992 $"
#

project(SampleVisionary)
cmake_minimum_required(VERSION 2.8)

### COMPILER FLAGS ###
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

### BUILD ###

include_directories(VisionaryCommon VisionaryCommon/include)
add_subdirectory(VisionaryCommon)

## Visionary-S sample ##

## Visionary-T sample ##
add_executable(SampleVisionaryT.exe SampleVisionaryT/SampleVisionaryT.cpp)
target_link_libraries(SampleVisionaryT.exe sick_visionary_common)

