
#ifndef __ObjectSegmentationSkeleton_WINDOW_H_
#define __ObjectSegmentationSkeleton_WINDOW_H_

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
#include "CGALSurfaceMesh.h"
#include "CGALSkeleton.h"
#include "Segmentation/Skeleton/MeshSkeleton.h"

#include <vector>

#include "ui_ObjectSegmentationSkeleton.h"

class ObjectSegmentationSkeletonWindow : public QMainWindow
{
    Q_OBJECT
public:
    ObjectSegmentationSkeletonWindow(const std::string& objectFile);
    ~ObjectSegmentationSkeletonWindow();

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
    void loadData();
    void reloadObject();
    void screenshot();

    void buildVisu();

    void buildObject();

    void renderDepthSkeletonImage();

protected:

    void loadObject();
    void setupUI();

    //static void timerCB(void* data, SoSensor* sensor);
    Ui::ObjectSegmentationSkeleton UI;
    SoQtExaminerViewer* viewer;

    SoSeparator* sceneSep;
    SoSeparator* objectSep;
    SoSeparator* skeletonSep;
    SoSeparator* segmentationSep;
    SoSeparator* surfaceSep;

    //std::string objectFile;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

    VirtualRobot::ManipulationObjectPtr manipObject;

    SimoxCGAL::MeshSkeletonPtr segSkeleton;
    SimoxCGAL::CGALSurfaceMeshPtr surfaceMesh;
    SimoxCGAL::CGALSkeletonPtr skeleton;
    std::string segmentedFilename;
    std::string objectFilename;
    std::string save_dir;
};


#endif
