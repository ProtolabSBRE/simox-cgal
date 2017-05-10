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
#include "CGALMeshConverter.h"

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
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
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
    int recTri = 0;
    if (trimesh)
        recTri = trimesh->faces.size();
    ss << "Nr Points: " << points.size() << "\nObject\n  Triangles: " << nrTriangles <<"\nReconstruction\n Triangles:" << recTri;

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
    if (object && UI.checkBoxObject->isChecked())
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

    reconstructionSep->removeAllChildren();
    if (reconstructedObject && UI.checkBoxReconstruction->isChecked())
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(reconstructedObject, colModel2);
        if (n)
        {
            SoMaterial* color = new SoMaterial();
            color->transparency = 0.7f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            reconstructionSep->addChild(color);
            reconstructionSep->addChild(n);
        }
    }


    pointsSep->removeAllChildren();
    if (points.size()>0 && UI.checkBoxPoints->isChecked())
    {
        SoNode* n = CoinVisualizationFactory::CreateVerticesVisualization(points, 2.0f);
        if (n)
        {
            SoMaterial* color = new SoMaterial();
            color->transparency = 0.7f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            pointsSep->addChild(color);
            pointsSep->addChild(n);
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
    t->normals.resize(size);
    for (int j = 0; j < faceCount; ++j)
    {
        MathTools::TriangleFace& face = t->faces.at(j);
        vertex2FaceMap[face.id1].insert(&t->faces.at(j));
        vertex2FaceMap[face.id2].insert(&t->faces.at(j));
        vertex2FaceMap[face.id3].insert(&t->faces.at(j));
    }
    int noNormals = 0;
    for (size_t i=0; i<vertex2FaceMap.size(); i++ )
    {
        std::set<MathTools::TriangleFace*> &fs = vertex2FaceMap.at(i);
        Eigen::Vector3f n;
        n.setZero();
        if (fs.size()==0)
        {
            //VR_WARNING << "No normal?!" << endl;
            noNormals++;
            n << 1.0f, 0, 0;
            t->normals[i] = n;
            continue;
        }

        for (MathTools::TriangleFace* tf: fs)
        {
            n += tf->normal;
        }
        n /= fs.size();
        t->normals[i] = n;
    }
    VR_INFO << "Created " << size-noNormals <<" normals. Skipped " << noNormals << " unconnected vertices." << endl;

    return true;
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
    reconstructedObject.reset();

    // extract points
    if (object && object->getVisualization() && object->getVisualization()->getTriMeshModel())
    {
        TriMeshModelPtr t = object->getVisualization()->getTriMeshModel();
        t->mergeVertices();
        if (t->vertices.size() != t->normals.size())
        {
            VR_INFO << "Updating normals, points.size = " << t->vertices.size() << " != normals.size=" << t->normals.size()<< endl;
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
    if (!object || points.size()==0)
        return;
    reconstruction.reset(new SimoxCGAL::MeshReconstruction());
    reconstruction->setVerbose(true);

    PolyhedronMeshPtr m = reconstruction->reconstructMesh(points, normals);
    CGALPolyhedronMeshPtr mesh(new CGALPolyhedronMesh(m));

    trimesh = CGALMeshConverter::ConvertCGALMesh(mesh);

    if (trimesh)
    {
        reconstructedObject = VirtualRobot::ManipulationObject::createFromMesh(trimesh);
    }
    buildVisu();
    updateInfo();
}

void MeshReconstructionWindow::save()
{
    if (!reconstructedObject)
    {
        return;
    }

    QString fi = QFileDialog::getSaveFileName(this, tr("Save ManipulationObject"), QString(), tr("XML Files (*.xml)"));
    std::string objectFile = std::string(fi.toLatin1());
    bool ok = false;

    try
    {
        boost::filesystem::path filenameBaseComplete(objectFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();
        boost::filesystem::path filenameBase = filenameBaseComplete.leaf();

        std::string fnSTL = filenameBase.stem().string() + ".stl";
        std::string fn = basePath + "/" + fnSTL;

        ObjectIO::writeSTL(trimesh,fn,reconstructedObject->getName());

        if (reconstructedObject->getVisualization())
            reconstructedObject->getVisualization()->setFilename(fnSTL, false);
        if (reconstructedObject->getCollisionModel() && reconstructedObject->getCollisionModel()->getVisualization())
            reconstructedObject->getCollisionModel()->getVisualization()->setFilename(fnSTL, false);
        ok = ObjectIO::saveManipulationObject(reconstructedObject, objectFile);
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
