
IF(Simox_USE_COIN_VISUALIZATION)
  #include(${CGAL_USE_FILE})
  SET(SRCS 
        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewer.cpp)
#        ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindowIO.cpp

  SET(INCS 
      ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.h
      )
#      ${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindowIO.h


  set(GUI_MOC_HDRS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewerWindow.h)

  set(GUI_UIS 
	${CMAKE_CURRENT_SOURCE_DIR}/SkeletonViewer.ui)

  SimoxQtApplication(SkeletonViewer "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  target_link_libraries(SkeletonViewer VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES})

ENDIF()
