cmake_minimum_required(VERSION 3.25)

project(OpenSimExtensionPlugin)

set(PLUGIN_NAME "osimExtensionPlugin"
    CACHE STRING "Name of shared library to create")

# OpenSim uses C++20 language features.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -march=native")
set(GCC_OPT_ENABLE "-funroll-loops")


# Find and hook up to OpenSim.
# ----------------------------
find_package(OpenSim REQUIRED PATHS "${OPENSIM_INSTALL_DIR}")

# Starting with version 3.3.1, Eigen provides a proper Config file.
# find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(BUILD_SHARED_LIBS "TRUE" CACHE BOOL "Build shared libraries." FORCE)

# Choose common build locations for executables and libraries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS} ./)

set (Plugin_OpenSim_Analyses_HEADERS
                        OpenSim/Extension/Analyses/osimAnalysesDLL.h
                        OpenSim/Extension/Analyses/RegisterTypes_osimAnalyses.h
                        OpenSim/Extension/Analyses/osimAnalyses.h
                        OpenSim/Extension/Analyses/DistanceKinematics.h
)

set (Plugin_OpenSim_Analyses_SOURCES
                        OpenSim/Extension/Analyses/osimAnalysesDLL.cpp
                        OpenSim/Extension/Analyses/RegisterTypes_osimAnalyses.cpp
                        OpenSim/Extension/Analyses/DistanceKinematics.cpp
)
set (Plugin_OpenSim_Common_HEADERS
                        OpenSim/Extension/Common/osimCommonDLL.h
                        OpenSim/Extension/Common/RegisterTypes_osimCommon.h
                        OpenSim/Extension/Common/osimCommon.h
                        OpenSim/Extension/Common/DistanceBaseDataReader.h
                        OpenSim/Extension/Common/DistanceDataReader.h
                        OpenSim/Extension/Common/DistanceDataReaderSettings.h
                        )

set (Plugin_OpenSim_Common_SOURCES
                        OpenSim/Extension/Common/osimCommonDLL.cpp
                        OpenSim/Extension/Common/RegisterTypes_osimCommon.cpp
                        OpenSim/Extension/Common/DistanceBaseDataReader.cpp
                        OpenSim/Extension/Common/DistanceDataReader.cpp
                        )

set (Plugin_OpenSim_Tools_HEADERS
                        OpenSim/Extension/Tools/osimToolsDLL.h
                        OpenSim/Extension/Tools/RegisterTypes_osimTools.h
                        OpenSim/Extension/Tools/osimTools.h
                        )


set (Plugin_OpenSim_Tools_SOURCES
                        OpenSim/Extension/Tools/osimToolsDLL.cpp
                        OpenSim/Extension/Tools/RegisterTypes_osimTools.cpp
                        )

set (Plugin_HEADERS
                        ${Plugin_OpenSim_Analyses_HEADERS}
                        ${Plugin_OpenSim_Common_HEADERS}
                        ${Plugin_OpenSim_Tools_HEADERS})

set (Plugin_SOURCES
                        ${Plugin_OpenSim_Common_SOURCES}
                        ${Plugin_OpenSim_Analyses_SOURCES}
                        ${Plugin_OpenSim_Tools_SOURCES}
)
# Build Shared Library
add_library(${PLUGIN_NAME} ${Plugin_HEADERS} ${Plugin_SOURCES} )
target_link_libraries(${PLUGIN_NAME} ${OpenSim_LIBRARIES})
target_compile_definitions(${PLUGIN_NAME} PRIVATE OSIMPLUGIN_EXPORTS)

set_target_properties(
    ${PLUGIN_NAME} PROPERTIES
    DEFINE_SYMBOL OSIMPLUGIN_EXPORTS
    PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

# install_targets(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})

# Build Demo Main
# If necessary, use the RELATIVE flag, otherwise each source file may be listed
# with full pathname. RELATIVE may makes it easier to extract an executable name
# automatically.
# file( GLOB APP_SOURCES RELATIVE app/*.cxx )
file( GLOB APP_SOURCES app/*.cpp )
foreach( testsourcefile ${APP_SOURCES} )
    # Cut off the file extension and directory path
    get_filename_component( testname ${testsourcefile} NAME_WE )
    add_executable( ${testname} ${testsourcefile} )
    # Make sure YourLib is linked to each app
    target_link_libraries( ${testname} ${PLUGIN_NAME})
    # This block copies the data files from data additional files into the running directory
    add_custom_command(TARGET ${testname} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/data/ $<TARGET_FILE_DIR:${testname}>)

endforeach( testsourcefile ${APP_SOURCES} )


