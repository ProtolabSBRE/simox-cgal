
#ifndef __SegmentObjectSDF_WINDOW_H_
#define __SegmentObjectSDF_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/ManipulationObject.h>
#include "SimoxCGAL.h"


#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <QGLWidget>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include "IO/SegmentedObjectIO.h"
#include "CGALPolyhedronMesh.h"
#include "Segmentation/SDF/MeshSDF.h"

#include <vector>

#include "ui_SegmentObjectSDF.h"

class SegmentObjectSDFWindow : public QMainWindow
{
    Q_OBJECT
public:
    SegmentObjectSDFWindow(const std::string& objectFile);
    ~SegmentObjectSDFWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt loop */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void colModel();

    void saveSegmentedObject();
    void reloadObject();
    void screenshot();

    void buildVisu();

    void buildObject();

protected:

    void loadObject();

    void setupUI();

    //static void timerCB(void* data, SoSensor* sensor);
    Ui::SegmentObjectSDF UI;
    SoQtExaminerViewer* viewer;

    SoSeparator* sceneSep;
    SoSeparator* objectSep;
    SoSeparator* segObjectSep;
    SoSeparator* sdfObjectSep;

    std::string objectFile;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

    VirtualRobot::ManipulationObjectPtr manipObject;

    SimoxCGAL::MeshSDFPtr sdfMesh;
    SimoxCGAL::CGALPolyhedronMeshPtr polyMesh;
    SimoxCGAL::SegmentedObjectPtr segObjectsPtr;
    std::string segmentedFilename;
    std::string objectFilename;
};


#endif
