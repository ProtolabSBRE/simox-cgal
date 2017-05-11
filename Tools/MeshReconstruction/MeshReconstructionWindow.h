
#ifndef __MESH_RECONSTRUCTION_WINDOW_H_
#define __SKELETON_GRASP_PLANNER_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/XML/rapidxml.hpp>

#include "MeshProcessing/MeshReconstruction.h"
#include "SimoxCGAL.h"
#include "SegmentedObject.h"
#include "CGALSurfaceMesh.h"

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>

#include <boost/foreach.hpp>

#include <vector>

#include "ui_MeshReconstruction.h"

// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

class MeshReconstructionWindow : public QMainWindow
{
    Q_OBJECT
public:
    MeshReconstructionWindow(std::string& objectFile);
    ~MeshReconstructionWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();

    void colModel();

    void buildVisu();

    void save();
    void loadPoints();
    void loadObject();

    void regularize();

    void doReconstruction();

protected:

    void loadObject(const std::string& objFilename);
    void setupUI();

    void updateInfo();

    bool updateNormals(VirtualRobot::TriMeshModelPtr t);


    SoSeparator* drawNormals(VirtualRobot::TriMeshModelPtr t);

    Ui::MeshReconstruction UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    std::string objectFile;

    SoSeparator* sceneSep;
    SoSeparator* pointsSep;
    SoSeparator* objectSep;
    SoSeparator* reconstructionSep;

    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::ManipulationObjectPtr reconstructedObject;
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector3f> normals;

    VirtualRobot::TriMeshModelPtr trimesh;

    SimoxCGAL::MeshReconstructionPtr reconstruction;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;
};

#endif
