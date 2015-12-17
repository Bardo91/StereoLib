cmake_minimum_required (VERSION 2.6)
if(POLICY CMP0048)
cmake_policy(SET CMP0048 OLD)

endif()
project(StereoLib)

# Collect all sources
file(GLOB_RECURSE STEREOLIB_SOURCE_FILES "StereoLib/*.cpp" "StereoLib/*.h" "StereoLib/*.inl")

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

GroupSources(StereoLib)

# Compiler specific options
if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

# Loading 3rd party libraries.
find_package(OpenCV 3 REQUIRED core imgproc ml highgui features2d xfeatures2d calib3d objdetect)
set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)
LINK_DIRECTORIES(${OpenCV_LIB_DIR})

find_package(PCL 1.2 REQUIRED common io kdtree filters visualization segmentation)
if(${PCL_FOUND} AND ${VTK_FOUND} AND ${Boost_FOUND} AND ${FLANN_FOUND} STREQUAL "TRUE")
	MESSAGE( STATUS " --> Adding properly PCL and its dependencies")
	add_definitions(-DENABLE_PCL)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
else()
	MESSAGE( FATAL_ERROR " --> Couldnt Add PCL and/or it's dependencies")
endif()

add_definitions(-D_CRT_SECURE_NO_WARNINGS)

# Library
add_library(StereoLib STATIC ${STEREOLIB_SOURCE_FILES})
target_include_directories(StereoLib PUBLIC ${PROJECT_SOURCE_DIR}/StereoLib)
target_include_directories(StereoLib PUBLIC ${OpenCV_INCLUDE_DIRS})
target_include_directories(StereoLib PUBLIC ${PROJECT_SOURCE_DIR}/../lib/cjson/src)
target_include_directories(StereoLib PUBLIC ${PROJECT_SOURCE_DIR}/../lib/libsvmpp/src)
target_include_directories(StereoLib PUBLIC ${PROJECT_SOURCE_DIR}/../lib/bovil/src)

target_link_libraries(StereoLib LINK_PUBLIC cjson)
target_link_libraries(StereoLib LINK_PUBLIC BOViL)
target_link_libraries(StereoLib LINK_PUBLIC libsvmpp)
target_link_libraries(StereoLib LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(StereoLib LINK_PUBLIC ${PCL_LIBRARIES})