
IF(Simox_USE_COIN_VISUALIZATION)
  #include(${CGAL_USE_FILE})
  SET(SRCS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.cpp 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewer.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonPolyhedron.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/IO/SkeletonIO.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonVisualization.cpp)

  SET(INCS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.h
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonPolyhedron.h
	${CMAKE_CURRENT_SOURCE_DIR}/IO/SkeletonIO.h
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonVisualization.h)

  set(GUI_MOC_HDRS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.h)

  set(GUI_UIS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewer.ui)

  SimoxQtApplication(SkeletonViewer "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  target_link_libraries(SkeletonViewer VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES})

ENDIF()
