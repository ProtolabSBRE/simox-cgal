
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
#include "Segmentation/Skeleton/Subpart.h"

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

    surfaceSep = new SoSeparator;
    sceneSep->addChild(surfaceSep);


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
    connect(UI.checkBoxSegment, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxSkeletonPoint, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonFullModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.radioButtonColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(reloadObject()));
    connect(UI.pushButtonBuild, SIGNAL(clicked()), this, SLOT(buildObject()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(saveSegmentedObject()));
    connect(UI.pushButtonScreenshot, SIGNAL(clicked()), this, SLOT(screenshot()));
    connect(UI.comboBoxSegmentation, SIGNAL(currentIndexChanged(int)), this, SLOT(buildVisu()));
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

    if (skeleton && UI.radioButtonFullModel->isChecked())
    {
        if (UI.checkBoxSkeleton->isChecked())
        {
            SoSeparator* s = new SoSeparator();
            SoMaterial* color = new SoMaterial();
            color->diffuseColor.setValue(1.f, 0.f, 0.f);
            s->addChild(color);
            s->addChild(SkeletonVisualization::createSkeletonVisualization(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.checkBoxLines->isChecked()));
            skeletonSep->addChild(s);
        }


        if (UI.checkBoxSkeletonPoint->isChecked())
        {
            skeletonSep->addChild(skeleton->showPoint(UI.spinBoxSkeletonPoint->value()));
        }

    }

    segmentationSep->removeAllChildren();


    int number_segmentation = UI.comboBoxSegmentation->count();
    int index_segmentation = UI.comboBoxSegmentation->currentIndex();
    bool lines = UI.checkBoxLines->isChecked();
    bool pigment = UI.checkBoxSegment->isChecked();

    SoMaterial* partColor = new SoMaterial;
    partColor->diffuseColor.setValue(1.f, 0.f, 0.f);


    if ((number_segmentation != 0) && UI.checkBoxManip->isChecked())
    {
        SkeletonPtr s = skeleton->getSkeleton();
        std::vector<ObjectPartPtr> members = segSkeleton->getSegmentedObject()->getObjectParts();

        if (index_segmentation < members.size())
        {
            segmentationSep->addChild(partColor);
            SubpartPtr subpart = boost::static_pointer_cast<Subpart>(members.at(index_segmentation));
            SoSeparator* segment = SkeletonVisualization::createSegmentVisualization(s, surfaceMesh->getMesh(), subpart, lines);
            segmentationSep->addChild(segment);

        } else if (index_segmentation == members.size()){

            SoSeparator* all = SkeletonVisualization::createSegmentationVisualization(s, surfaceMesh->getMesh(), members, lines);
            segmentationSep->addChild(all);
        }
    }

    surfaceSep->removeAllChildren();
    if (pigment)
    {
        SkeletonPtr s = skeleton->getSkeleton();
        std::vector<ObjectPartPtr> members = segSkeleton->getSegmentedObject()->getObjectParts();

        if (index_segmentation < members.size())
        {
//            surfaceSep->addChild(partColor);
            SubpartPtr subpart = boost::static_pointer_cast<Subpart>(members.at(index_segmentation));
            SoNode* segment = SkeletonVisualization::createPigmentedSubpartVisualization(s, surfaceMesh->getMesh(), subpart, VirtualRobot::VisualizationFactory::Color(1.f, 0.f, 0.f));
            surfaceSep->addChild(segment);

        } else if (index_segmentation == members.size()){

            SoSeparator* all = SkeletonVisualization::createPigmentedMeshVisualization(s, surfaceMesh->getMesh(), members, members.size());
            surfaceSep->addChild(all);
        }

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
    segSkeleton.reset();
    surfaceMesh.reset();
    skeleton.reset();

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

    VR_INFO << "Done in " << skeleton->getTime() << " ms " << endl;


    VR_INFO << "Calculatin skeleton segmentation ..." << endl;

    segSkeleton = MeshSkeletonPtr(new MeshSkeleton(surfaceMesh, skeleton->getSkeleton(), 20.0));

    VR_INFO << "Done in " << segSkeleton->getTime() << " ms " << endl;


    UI.comboBoxSegmentation->clear();
    vector<ObjectPartPtr> seg = segSkeleton->getSegmentedObject()->getObjectParts();
    for (int i = 0; i < segSkeleton->getSegmentedObject()->getObjectParts().size(); i++)
    {

        SubpartPtr tmp = boost::static_pointer_cast<Subpart>(seg.at(i));
        string s = tmp->name;
        UI.comboBoxSegmentation->addItem(QString(s.c_str()));
    }

    UI.comboBoxSegmentation->addItem(QString("All segment"));
    UI.comboBoxSegmentation->addItem(QString("No segment"));

    UI.comboBoxSegmentation->setCurrentIndex(segSkeleton->getSegmentedObject()->getObjectParts().size() + 1);
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

