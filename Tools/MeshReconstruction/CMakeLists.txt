PROJECT ( MeshReconstruction )

SET(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshReconstructionWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshReconstruction.cpp)

SET(INCS
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshReconstructionWindow.h)

set(GUI_MOC_HDRS
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshReconstructionWindow.h)

set(GUI_UIS
    ${CMAKE_CURRENT_SOURCE_DIR}/MeshReconstruction.ui)

SimoxQtApplication(${PROJECT_NAME} "${SRCS}" "${INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
target_link_libraries(${PROJECT_NAME} PRIVATE simox-cgal)

  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox-CGAL_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Tools")

