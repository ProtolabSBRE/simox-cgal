PROJECT ( SkeletonGraspPlanner )

SET(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlannerWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanner.cpp)

SET(INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlannerWindow.h)

set(GUI_MOC_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlannerWindow.h)

set(GUI_UIS
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanner.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlannerOptions.ui
    )

SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
target_link_libraries(${PROJECT_NAME} VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES})

  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox-CGAL_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Tools")

