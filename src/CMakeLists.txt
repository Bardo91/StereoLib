cmake_minimum_required (VERSION 2.6)
if(POLICY CMP0048)
cmake_policy(SET CMP0048 OLD)
endif()
project(StereoLib)

# Collect all sources
file(GLOB_RECURSE BOVIL_SOURCE_FILES "code/*.cpp" "code/*.h" "code/*.inl")

# Clasify sources according to folder structure.
# This creates a nice layout for the project in visual studio
# This macro is courtesy of http://www.cmake.org/pipermail/cmake/2013-November/056336.html
macro(GroupSources curdir)
	file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
		${PROJECT_SOURCE_DIR}/${curdir}/*)
	foreach(child ${children})
		if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
			GroupSources(${curdir}/${child})
		else()
			string(REPLACE "/" "\\" groupname ${curdir})
			source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
		endif()
	endforeach()
endmacro()

GroupSources(code)

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Library
add_library(StereoLib STATIC ${BOVIL_SOURCE_FILES})
target_include_directories(StereoLib PUBLIC ${PROJECT_SOURCE_DIR}/code)