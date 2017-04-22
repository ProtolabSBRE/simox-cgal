
#ifndef __SKELETON_GRASP_PLANNER_WINDOW_H_
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

#include "GraspPlanning/GraspStudio.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "GraspPlanning/GraspPlanner/GenericGraspPlanner.h"

#include "GraspPlanning/Skeleton/SkeletonGraspPlanner.h"
#include "GraspPlanning/Skeleton/ApproachMovementSkeleton.h"
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

#include "ui_SkeletonGraspPlanner.h"

// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

class SkeletonGraspPlannerWindow : public QMainWindow
{
    Q_OBJECT
public:
    SkeletonGraspPlannerWindow(std::string& robotFile, std::string& eefName, std::string& preshape, std::string& segmentedObjectFile);
    ~SkeletonGraspPlannerWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void colModel();
    void frictionConeVisu();
    void showGrasps();

    void buildVisu();

    void plan();
    void planAll();
    void save();
    void loadData();

    void selectGrasp();

    void setVerbose();

protected:

    void loadRobot();
    void loadSegmentedObject(const std::string & filename);

    void planGrasps(float timeout, bool forceClosure, float quality, int nrGrasps);
    void initPlanner();

    void setupUI();

    void updateSkeletonInfo();

    static void timerCB(void* data, SoSensor* sensor);
    Ui::SkeletonGraspPlanner UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* graspsSep;
    SoSeparator* skeletonSep;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr eefCloned;
//    VirtualRobot::ObstaclePtr object;
    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::EndEffectorPtr eef;

    VirtualRobot::GraspSetPtr grasps;


    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string segmentedObjectFile;
    std::string eefName;
    std::string preshape;

    SoSeparator* eefVisu;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
    SimoxCGAL::ApproachMovementSkeletonPtr approach;
    SimoxCGAL::SkeletonGraspPlannerPtr planner;


    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

    //data for skeleton
    VirtualRobot::TriMeshModelPtr triMeshRefined;
    SimoxCGAL::SkeletonPtr skeleton;
    SimoxCGAL::SegmentedObjectPtr segmentation;
    SimoxCGAL::CGALSurfaceMeshPtr mesh;


    //test
    SoSeparator* test;
};

#endif
