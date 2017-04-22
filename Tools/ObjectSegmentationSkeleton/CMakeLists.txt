
IF(Simox_USE_COIN_VISUALIZATION)
  #include(${CGAL_USE_FILE})
  SET(SRCS 
        ${CMAKE_CURRENT_SOURCE_DIR}/ObjectSegmentationSkeletonWindow.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ObjectSegmentationSkeleton.cpp)

  SET(INCS 
      ${CMAKE_CURRENT_SOURCE_DIR}/ObjectSegmentationSkeletonWindow.h
      )


  set(GUI_MOC_HDRS 
        ${CMAKE_CURRENT_SOURCE_DIR}/ObjectSegmentationSkeletonWindow.h)

  set(GUI_UIS 
        ${CMAKE_CURRENT_SOURCE_DIR}/ObjectSegmentationSkeleton.ui)

  SimoxQtApplication(ObjectSegmentationSkeleton "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  target_link_libraries(ObjectSegmentationSkeleton VirtualRobot simox-cgal ${Simox_EXTERNAL_LIBRARIES} ${CGAL_LIBRARIES})

ENDIF()
