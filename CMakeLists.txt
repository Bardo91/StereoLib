cmake_minimum_required (VERSION 2.6)
if(POLICY CMP0048)
	cmake_policy(SET CMP0048 OLD)
endif()

project(StereoLib)
set(PROJECT_VERSION "alpha_1.0.0")
enable_testing()

# Generade documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
	add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Generating API documentation with Doxygen" VERBATIM)
endif(DOXYGEN_FOUND)

# Library Projects
add_subdirectory(src)
