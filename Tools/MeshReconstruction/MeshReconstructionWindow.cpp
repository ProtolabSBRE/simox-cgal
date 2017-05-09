#include "MeshReconstructionWindow.h"
#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/ContactConeGenerator.h"
#include "GraspPlanning/MeshConverter.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "VirtualRobot/Visualization/TriMeshModel.h"
#include "VirtualRobot/Visualization/VisualizationFactory.h"
#include <VirtualRobot/Import/RobotImporterFactory.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <QFileDialog>
#include <QObject>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoMaterial.h>

#include "Visualization/CoinVisualization/CGALCoinVisualization.h"

#include <sstream>


using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

MeshReconstructionWindow::MeshReconstructionWindow(std::string& objectFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    // init the random number generator
    srand(time(NULL));

    this->objectFile = objectFile;


    sceneSep = new SoSeparator;
    sceneSep->ref();
    objectSep = new SoSeparator;
    sceneSep->addChild(objectSep);
    pointsSep = new SoSeparator;
    sceneSep->addChild(pointsSep);
    reconstructionSep = new SoSeparator;
    sceneSep->addChild(reconstructionSep);
    setupUI();

    loadObject(objectFile);

    buildVisu();
    viewer->viewAll();
}


MeshReconstructionWindow::~MeshReconstructionWindow()
{
    sceneSep->unref();
}



void MeshReconstructionWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));


    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();
    viewer->setAccumulationBuffer(true);
    viewer->setAntialiasing(true, 4);

    connect(UI.pushButtonLoadPoints, SIGNAL(clicked()), this, SLOT(loadPoints()));
    connect(UI.pushButtonReconstruct, SIGNAL(clicked()), this, SLOT(doReconstruction()));
    connect(UI.pushButtonLoadObject, SIGNAL(clicked()), this, SLOT(loadObject()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxObject, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxPoints, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxReconstruction, SIGNAL(clicked()), this, SLOT(buildVisu()));

}

void MeshReconstructionWindow::updateInfo()
{
    std::stringstream ss;
    ss << std::setprecision(3);
    int nrTriangles = 0;
    if (object && object->getVisualization() && object->getVisualization()->getTriMeshModel())
        nrTriangles = object->getVisualization()->getTriMeshModel()->faces.size();
    ss << "Nr Points: " << points.size() << "\nObject\n  Triangles: " << nrTriangles <<"\n";

    UI.labelInfo->setText(QString(ss.str().c_str()));
}


void MeshReconstructionWindow::resetSceneryAll()
{
    updateInfo();
}


void MeshReconstructionWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void MeshReconstructionWindow::buildVisu()
{
    objectSep->removeAllChildren();
    if (object)
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;       
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(object, colModel2);
        if (n)
        {
            SoMaterial* color = new SoMaterial();
            color->transparency = 0.7f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            objectSep->addChild(color);
            objectSep->addChild(n);
        }
    }
    viewer->scheduleRedraw();
}

int MeshReconstructionWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void MeshReconstructionWindow::quit()
{
    std::cout << "GraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void MeshReconstructionWindow::loadPoints()
{
}

void MeshReconstructionWindow::loadObject()
{
    resetSceneryAll();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("XML Files (*.xml)"));
    string file = std::string(fi.toAscii());
    if (file.empty())
    {
        return;
    }
    loadObject(file);
}

bool MeshReconstructionWindow::updateNormals(TriMeshModelPtr t)
{
    if (!t)
        return false;
    int size = t->vertices.size();
    int faceCount = t->faces.size();
    std::vector<std::set<MathTools::TriangleFace*>> vertex2FaceMap(size);
    for (int j = 0; j < faceCount; ++j)
    {
        MathTools::TriangleFace& face = t->faces.at(j);
        vertex2FaceMap[face.id1].insert(&t->faces.at(j));
        vertex2FaceMap[face.id2].insert(&t->faces.at(j));
        vertex2FaceMap[face.id3].insert(&t->faces.at(j));
    }
    // todo...

    return false;
}

void MeshReconstructionWindow::loadObject(const std::string & filename)
{
    objectFile = filename;

    try
    {
        object = ObjectIO::loadManipulationObject(objectFile);
    } catch (...)
    {
        VR_ERROR << "could not load file " << objectFile << endl;
        return;
    }
    if (!object)
        return;

    trimesh.reset();
    points.clear();
    normals.clear();

    // extract points
    if (object && object->getVisualization() && object->getVisualization()->getTriMeshModel())
    {
        TriMeshModelPtr t = object->getVisualization()->getTriMeshModel();
        t->mergeVertices();
        if (t->vertices.size() != t->normals.size())
        {
            VR_ERROR << "Updating normals, since points.size = " << t->vertices.size() << " != normals.size=" << t->normals.size()<< endl;
            if (!updateNormals(t))
            {
                object.reset();
                buildVisu();
                return;
            }
        }
        points = t->vertices;
        normals = t->normals;
    }

    buildVisu();

    updateInfo();
}

void MeshReconstructionWindow::colModel()
{
    buildVisu();
}

void MeshReconstructionWindow::doReconstruction()
{
}

void MeshReconstructionWindow::save()
{
    if (!trimesh)
    {
        return;
    }

    ManipulationObjectPtr objectM = VirtualRobot::ManipulationObject::createFromMesh(trimesh);
    QString fi = QFileDialog::getSaveFileName(this, tr("Save ManipulationObject"), QString(), tr("XML Files (*.xml)"));
    std::string objectFile = std::string(fi.toLatin1());
    bool ok = false;

    try
    {
        ok = ObjectIO::saveManipulationObject(objectM, objectFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while saving object" << endl;
        cout << e.what();
        return;
    }

    if (!ok)
    {
        cout << " ERROR while saving object" << endl;
        return;
    }
}
