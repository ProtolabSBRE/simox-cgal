cmake_minimum_required(VERSION 2.8.12)

project(simox-cgal)


########################################
###          Find packages           ###
########################################

find_package(Simox 2.3.48 REQUIRED)
find_package(CGAL REQUIRED)
FIND_PACKAGE(Boost 1.46.0 COMPONENTS unit_test_framework REQUIRED)
SET (Boost_TEST_LIB "${Boost_LIBRARIES}")
# disable boost auto linking

setupSimoxExternalLibraries()
include(${CGAL_USE_FILE})
include_directories (SYSTEM ${CGAL_INCLUDE_DIRS})

set (SimoxCGAL_BUILD_Tests TRUE CACHE BOOL "Build tests")
set (SimoxCGAL_BUILD_Tools TRUE CACHE BOOL "Build tools")


if (NOT Simox_USE_COIN_VISUALIZATION)
    message( FATAL_ERROR "SIMOX must be compiled with COIN3D support, exiting..." )
endif()

########################################
###      Define files for build      ###
########################################

# (glob should not be used here, for rationale see https://cmake.org/cmake/help/v3.0/command/file.html @ "We do not recommend using GLOB to...")
set(SOURCE_FILES
    SimoxCGAL.cpp
    CGALSurfaceMesh.cpp
    CGALPolyhedronMesh.cpp
    CGALMeshConverter.cpp
    IO/CGALMeshIO.cpp
    IO/SegmentedObjectIO.cpp
    IO/SkeletonIO.cpp
    SegmentedObject.cpp
    ObjectPart.cpp
    CGALSkeleton.cpp
    Segmentation/Skeleton/MeshSkeleton.cpp
    Segmentation/Skeleton/MeshSkeletonData.cpp
    Segmentation/Skeleton/SkeletonPart.cpp
    Segmentation/SDF/MeshSDF.cpp
    MeshProcessing/MeshReconstruction.h
    Visualization/CoinVisualization/CGALCoinVisualization.cpp
    GraspPlanning/Skeleton/SkeletonVertexAnalyzer.cpp
    GraspPlanning/Skeleton/ApproachMovementSkeleton.cpp
    GraspPlanning/Skeleton/DeciderGraspPreshape.cpp
    GraspPlanning/Skeleton/SkeletonGraspPlanner.cpp
    )
set(HEADER_FILES
    SimoxCGAL.h
    SimoxCGALImportExport.h
    CGALSurfaceMesh.h
    CGALPolyhedronMesh.h
    CGALMeshConverter.h
    IO/CGALMeshIO.h
    IO/SkeletonIO.h
    IO/SegmentedObjectIO.h
    SegmentedObject.h
    ObjectPart.h
    CGALSkeleton.h
    Segmentation/Skeleton/MeshSkeleton.h
    Segmentation/Skeleton/MeshSkeletonData.h
    Segmentation/Skeleton/SkeletonPart.h
    Segmentation/Skeleton/SkeletonPoint.h
    Segmentation/SDF/MeshSDF.h
    MeshProcessing/MeshReconstruction.cpp
    Visualization/CoinVisualization/CGALCoinVisualization.h
    GraspPlanning/Skeleton/SkeletonVertexAnalyzer.h
    GraspPlanning/Skeleton/ApproachMovementSkeleton.h
    GraspPlanning/Skeleton/DeciderGraspPreshape.h
    GraspPlanning/Skeleton/SkeletonGraspPlanner.h
    )

#######################################################################
# OS specific options
#######################################################################
IF(UNIX)
    # We are on Linux
    SET(SimoxCGAL_TEST_DIR ${CMAKE_BINARY_DIR}/tests)
ELSE(UNIX)
    # We are on Windows
    SET(SimoxCGAL_TEST_DIR ${CMAKE_BINARY_DIR})
    # On MSVC we compile with /MP flag (use multiple threads)
    IF(MSVC)
        ADD_DEFINITIONS(/MP)
    ENDIF(MSVC)
ENDIF(UNIX)

#######################################################################
# Setup for testing
#######################################################################
ENABLE_TESTING()
INCLUDE(CTest)
MESSAGE(STATUS "** Test output directory: ${SimoxCGAL_TEST_DIR}")
#ADD_DEFINITIONS(-DSimoxCGAL_TEST_DIR=\"${SimoxCGAL_TEST_DIR}/\")
MACRO(ADD_SIMOX_CGAL_TEST TEST_NAME)
    INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})

    if (NOT Boost_USE_STATIC_LIBS)
        # enable dynamic linking for boost test libraries
        ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)
    endif ()
    ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})
    add_definitions(-DCGAL_EIGEN3_ENABLED)
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES} ${Boost_TEST_LIB})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${SimoxCGAL_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "SimoxCGAL Tests")
    ADD_TEST(NAME SimoxCGAL_${TEST_NAME}
        COMMAND ${SimoxCGAL_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO()

if(SimoxCGAL_BUILD_Tests)
    # include unit tests
    ADD_SUBDIRECTORY(tests)
endif()
########################################
###           Setup dirs             ###
########################################
SET(Simox-CGAL_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
SET(BIN_DIR bin)
SET(LIB_DIR lib)
SET(DATA_DIR data)

SET(Simox-CGAL_LIB_DIR ${Simox-CGAL_BUILD_DIRECTORY}/${LIB_DIR})
SET(Simox-CGAL_BIN_DIR ${Simox-CGAL_BUILD_DIRECTORY}/${BIN_DIR})

# todo: installation data dir
GET_FILENAME_COMPONENT (CurrentSimoxCGALPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(Simox-CGAL_DATA_DIR ${CurrentSimoxCGALPath}/data)
ADD_DEFINITIONS("-DSimox_CGAL_DATA_PATH=\"${Simox-CGAL_DATA_DIR}\"")

MESSAGE (STATUS "** SIMOX-CGAL DIR: ${CurrentSimoxCGALPath}")
MESSAGE (STATUS "** SIMOX-CGAL LIB DIR: ${Simox-CGAL_LIB_DIR}")
MESSAGE (STATUS "** SIMOX-CGAL BIN DIR: ${Simox-CGAL_BIN_DIR}")
MESSAGE (STATUS "** SIMOX-CGAL DATA DIR: ${Simox-CGAL_DATA_DIR}")


########################################
###           Setup build            ###
########################################

# Setup build
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} VirtualRobot GraspStudio ${CGAL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox-CGAL_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox-CGAL_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox-CGAL_LIB_DIR})

set(CMAKE_POSITION_INDEPENDENT_CODE ON) # enable -fPIC
add_definitions(-DCGAL_EIGEN3_ENABLED)

# Enables C++11 and additional warnings (if desired, you can customize this)
target_compile_options(${PROJECT_NAME} PUBLIC "-std=c++11" "-Wall" "-Wextra" "-Wno-long-long") #"-pedantic"

# Automatically put include path for dependant projects that are linked against this lib
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)


########################################
###           Setup tools            ###
########################################

if(SimoxCGAL_BUILD_Tools)
    # include unit tests
    ADD_SUBDIRECTORY(Tools)
endif()

########################################
###    Version support               ###
########################################

set(PROJ_VERSION 0.0.1)
set(PROJ_SO_VERSION 1) # shared lib (.so file) build number
set_property(TARGET ${PROJECT_NAME} PROPERTY VERSION ${PROJ_VERSION})
set_property(TARGET ${PROJECT_NAME} PROPERTY SOVERSION ${PROJ_SO_VERSION})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
     VERSION ${PROJ_VERSION}
     COMPATIBILITY AnyNewerVersion
)


########################################
###          Export targets          ###
########################################

# Export targets so that SimpleH2TLibrary can be found using find_package()
export(PACKAGE ${PROJECT_NAME})
export(TARGETS ${PROJECT_NAME} FILE "${CMAKE_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")  # If there are several targets (e.g., several libraries), you need to list them all here

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMakeModules/")

#configure_file(
#    ${CMAKE_MODULE_PATH}/simox-cgalConfig.cmake
#    ${CMAKE_BINARY_DIR}/simox-cgalConfig.cmake
#    COPYONLY
#)


########################################
###      Install configuration       ###
########################################

set(INSTALL_CMAKE_DIR "share/cmake/${PROJECT_NAME}")

install(
    TARGETS "${PROJECT_NAME}"
    EXPORT "${PROJECT_NAME}Targets"
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(
    EXPORT "${PROJECT_NAME}Targets"
    FILE "${PROJECT_NAME}Targets.cmake"
    DESTINATION ${INSTALL_CMAKE_DIR}
)

install(
    FILES
        "CMakeModules/${PROJECT_NAME}Config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${INSTALL_CMAKE_DIR}
    COMPONENT Devel
)

install (FILES ${HEADER_FILES} DESTINATION include)
