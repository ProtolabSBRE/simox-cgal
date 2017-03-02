
#include "SkeletonViewerWindow.h"
//#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
//#include "GraspPlanning/ContactConeGenerator.h"
//#include "VirtualRobot/EndEffector/EndEffector.h"
//#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/ManipulationObject.h>
//#include "VirtualRobot/Grasping/Grasp.h"
//#include "VirtualRobot/IK/GenericIKSolver.h"
//#include "VirtualRobot/Grasping/GraspSet.h"
//#include "VirtualRobot/CollisionDetection/CDManager.h"
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <QFileDialog>
#include <QInputDialog>
#include <Eigen/Geometry>
#include "CGALMeshConverter.h"

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>



#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>

#include "SkeletonVisualization.h"

#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

SkeletonViewerWindow::SkeletonViewerWindow(const std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->objectFilename = objFile;

    //boost::filesystem::path p(objFile);
    //save_dir = p.parent_path().string();

    sceneSep = new SoSeparator;
    sceneSep->ref();

    objectSep = new SoSeparator;
    sceneSep->addChild(objectSep);

    skeletonSep = new SoSeparator;
    sceneSep->addChild(skeletonSep);

    segmentationSep = new SoSeparator;
    sceneSep->addChild(segmentationSep);


    setupUI();

    loadObject();
    buildVisu();
    viewer->viewAll();
}


SkeletonViewerWindow::~SkeletonViewerWindow()
{
    sceneSep->unref();
}

void SkeletonViewerWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
    viewer->setAntialiasing(true, 8);
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();
    viewer->setAntialiasing(true, 8);

    connect(UI.checkBoxManip, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxSkeleton, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxLines, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonFullModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.radioButtonColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(reloadObject()));
    connect(UI.pushButtonBuild, SIGNAL(clicked()), this, SLOT(buildObject()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(saveSegmentedObject()));
    connect(UI.pushButtonScreenshot, SIGNAL(clicked()), this, SLOT(screenshot()));
}


void SkeletonViewerWindow::resetSceneryAll()
{
}


void SkeletonViewerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SkeletonViewerWindow::buildVisu()
{
    SceneObject::VisualizationType colModel = (UI.radioButtonColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    objectSep->removeAllChildren();
    if (manipObject && UI.checkBoxManip->isChecked())
    {
//        SoNode* n = CoinVisualizationFactory::getCoinVisualization(manipObject, colModel);
        visualizationObject = manipObject->getVisualization<CoinVisualization>();
        visualizationObject->setTransparency(0.7f);
        SoNode* n = visualizationObject->getCoinVisualization();

        if (n)
        {
            objectSep->addChild(n);
        }

    }

    skeletonSep->removeAllChildren();

    if (skeleton && UI.checkBoxSkeleton->isChecked() && UI.radioButtonFullModel->isChecked())
    {
        SoSeparator* s = new SoSeparator();
        SoMaterial* color = new SoMaterial();
        color->diffuseColor.setValue(1.f, 0.f, 0.f);
        s->addChild(color);
        s->addChild(SkeletonVisualization::createSkeletonVisualization(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.checkBoxLines->isChecked()));
        skeletonSep->addChild(s);

    }

    viewer->scheduleRedraw();
}

int SkeletonViewerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void SkeletonViewerWindow::quit()
{
    std::cout << "SkeletonViewer: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void SkeletonViewerWindow::saveSegmentedObject()
{

}

void SkeletonViewerWindow::loadObject()
{
    manipObject = ObjectIO::loadManipulationObject(objectFilename);

    buildVisu();
    viewer->viewAll();
}


void SkeletonViewerWindow::colModel()
{
    buildVisu();
}


void SkeletonViewerWindow::reloadObject()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("XML Files (*.xml)"));
    objectFilename = std::string(fi.toAscii());
    if (objectFilename.empty())
    {
        return;
    }

    loadObject();
}


void SkeletonViewerWindow::buildObject()
{
    if (!manipObject)
    {
        VR_ERROR << "no manipulation obejct" << endl;
        return;
    }
    if (!manipObject->getCollisionModel())
    {
        VR_ERROR << "no collision model of manipulation obejct" << endl;
        return;
    }

    VR_INFO << "Converting mesh to cgal structure..." << endl;

    surfaceMesh = CGALMeshConverter::ConvertToSurfaceMesh(manipObject->getCollisionModel()->getTriMeshModel());

    VR_INFO << "Calculatin skeleton ..." << endl;

    skeleton = SkeletonPolyhedronPtr(new SkeletonPolyhedron(manipObject->getName(), surfaceMesh->getMesh()));
    skeleton->initParameters();
    skeleton->calculateSkeleton();

    VR_INFO << "done." << endl;

    //create segmentation

//    VR_INFO << "done." << endl;
}

void SkeletonViewerWindow::screenshot()
{
    //SoCamera* camera = viewer->getCamera();
    //Fit to object representation
    //camera->orientation.setValue(SbVec3f(0,1,1),1.5707963f); //hammer
    //camera->orientation.setValue(SbVec3f(0,0,1),0.7853981f); //teddy
    //camera->orientation.setValue(SbVec3f(0,1,0),1.57079f); //screwdriver
    //viewer->viewAll();

    QString fi = QFileDialog::getSaveFileName(this, tr("Save Screenshot"));
    std::string buffer = std::string(fi.toAscii());
    if (buffer.empty())
    {
        return;
    }

    SbString framefile;
    framefile.sprintf(buffer.c_str(), 1);

    viewer->getSceneManager()->render();
    viewer->getSceneManager()->scheduleRedraw();
    QGLWidget* w = (QGLWidget*)viewer->getGLWidget();

    QImage i = w->grabFrameBuffer();
    bool bRes = i.save(framefile.getString(), "PNG");
    if (bRes)
    {
        cout << "wrote image " << endl;
    }
    else
    {
        cout << "failed writing image " << endl;
    }

}

