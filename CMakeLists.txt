cmake_minimum_required(VERSION 2.8.12)

project(simox-cgal)


########################################
###          Find packages           ###
########################################

find_package(Simox REQUIRED)
find_package(CGAL REQUIRED)

setupSimoxExternalLibraries()
include(${CGAL_USE_FILE})

########################################
###      Define files for build      ###
########################################

# (glob should not be used here, for rationale see https://cmake.org/cmake/help/v3.0/command/file.html @ "We do not recommend using GLOB to...")
set(SOURCE_FILES
    CGALMesh.cpp
    CGALMeshIO.cpp
    CGALMeshConverter.cpp
    SegmentedObjectIO.cpp
    SegmentedObject.cpp
    ObjectPart.cpp
    )
set(HEADER_FILES
    SimoxCGAL.h
    SimoxCGALImportExport.h
    CGALMesh.h
    CGALMeshIO.h
    CGALMeshConverter.h
    SegmentedObjectIO.h
    SegmentedObject.h
    ObjectPart.h
    )


########################################
###           Setup build            ###
########################################

# Setup build
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} VirtualRobot ${CGAL_LIBRARIES})

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
