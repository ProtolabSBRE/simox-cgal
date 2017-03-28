
IF(Simox_USE_COIN_VISUALIZATION)
  #include(${CGAL_USE_FILE})
  SET(SRCS 
        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanerWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlaner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/SkeletonVertexAnalyzer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/PartAnalysis.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/ApproachMovementSkeleton.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/DeciderGraspPreshape.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/SkeletonGraspPlaner.cpp)

    #        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanerViewerIO.cpp

  SET(INCS 
     	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanerWindow.h
        ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/SkeletonVertexAnalyzer.h
        ${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/PartAnalysis.h
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/ApproachMovementSkeleton.h
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/DeciderGraspPreshape.h
	${CMAKE_CURRENT_SOURCE_DIR}/GraspPlaner/SkeletonGraspPlaner.h)
#      	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanerViewerIO.h


  set(GUI_MOC_HDRS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlanerWindow.h)

  set(GUI_UIS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonGraspPlaner.ui)

  SimoxQtApplication(SkeletonGraspPlaner "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  target_link_libraries(SkeletonGraspPlaner VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES})

ENDIF()
