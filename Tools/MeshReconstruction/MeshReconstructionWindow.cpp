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
    connect(UI.checkBoxNormalsReconstr, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxNormalsObj, SIGNAL(clicked()), this, SLOT(buildVisu()));

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


SoSeparator* MeshReconstructionWindow::drawNormals(TriMeshModelPtr t)
{
    SoSeparator* res = new SoSeparator;
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    res->addChild(u);
    Eigen::Vector3f z(0, 0, 1.0f);
    SoSeparator* arrow = CoinVisualizationFactory::CreateArrow(z, 10.0f, 0.8f);
    arrow->ref();

    if (t->normals.size() > 0)
    {
        for (size_t i = 0; i < t->faces.size(); i++)
        {
            unsigned int id1 = t->faces[i].id1;
            unsigned int id2 = t->faces[i].id2;
            unsigned int id3 = t->faces[i].id3;
            Eigen::Vector3f &v1 = t->vertices[id1];
            Eigen::Vector3f &v2 = t->vertices[id2];
            Eigen::Vector3f &v3 = t->vertices[id3];

            unsigned int normalIndx1 = t->faces[i].idNormal1;
            unsigned int normalIndx2 = t->faces[i].idNormal2;
            unsigned int normalIndx3 = t->faces[i].idNormal3;
            Eigen::Vector3f &normal1 = t->normals[normalIndx1];
            Eigen::Vector3f &normal2 = t->normals[normalIndx2];
            Eigen::Vector3f &normal3 = t->normals[normalIndx3];

            if (fabs(normal1.norm() - 1.0f) > 1.1)
            {
                VR_ERROR << "Wrong normal, norm:" << normal1.norm() << endl;
            }

            if (fabs(normal2.norm() - 1.0f) > 1.1)
            {
                VR_ERROR << "Wrong normal, norm:" << normal2.norm() << endl;
            }

            if (fabs(normal3.norm() - 1.0f) > 1.1)
            {
                VR_ERROR << "Wrong normal, norm:" << normal3.norm() << endl;
            }

            SoMatrixTransform* mt1 = new SoMatrixTransform;
            SoMatrixTransform* mt2 = new SoMatrixTransform;
            SoMatrixTransform* mt3 = new SoMatrixTransform;

            MathTools::Quaternion q1 = MathTools::getRotation(z, normal1);
            MathTools::Quaternion q2 = MathTools::getRotation(z, normal2);
            MathTools::Quaternion q3 = MathTools::getRotation(z, normal3);
            Eigen::Matrix4f mat1 = MathTools::quat2eigen4f(q1);
            Eigen::Matrix4f mat2 = MathTools::quat2eigen4f(q2);
            Eigen::Matrix4f mat3 = MathTools::quat2eigen4f(q3);
            mat1.block(0, 3, 3, 1) = v1;
            mat2.block(0, 3, 3, 1) = v2;
            mat3.block(0, 3, 3, 1) = v3;
            SbMatrix m1(reinterpret_cast<SbMat*>(mat1.data()));
            SbMatrix m2(reinterpret_cast<SbMat*>(mat2.data()));
            SbMatrix m3(reinterpret_cast<SbMat*>(mat3.data()));
            mt1->matrix.setValue(m1);
            mt2->matrix.setValue(m2);
            mt3->matrix.setValue(m3);
            SoSeparator* sn1 = new SoSeparator();
            sn1->addChild(mt1);
            sn1->addChild(arrow);
            res->addChild(sn1);
            SoSeparator* sn2 = new SoSeparator();
            sn2->addChild(mt2);
            sn2->addChild(arrow);
            res->addChild(sn2);
            SoSeparator* sn3 = new SoSeparator();
            sn3->addChild(mt3);
            sn3->addChild(arrow);
            res->addChild(sn3);
        }
    } else
    {
        for (size_t i = 0; i < t->faces.size(); i++)
        {
            unsigned int id1 = t->faces[i].id1;
            unsigned int id2 = t->faces[i].id2;
            unsigned int id3 = t->faces[i].id3;
            Eigen::Vector3f &v1 = t->vertices[id1];
            Eigen::Vector3f &v2 = t->vertices[id2];
            Eigen::Vector3f &v3 = t->vertices[id3];
            Eigen::Vector3f v = (v1+v2+v3)/3.0f;

            Eigen::Vector3f &normal1 = t->faces[i].normal;

            if (fabs(normal1.norm() - 1.0f) > 1.1)
            {
                VR_ERROR << "Wrong normal, norm:" << normal1.norm() << endl;
                continue;
            }


            SoMatrixTransform* mt1 = new SoMatrixTransform;
            MathTools::Quaternion q1 = MathTools::getRotation(z, normal1);
            Eigen::Matrix4f mat1 = MathTools::quat2eigen4f(q1);
            mat1.block(0, 3, 3, 1) = v;
            SbMatrix m1(reinterpret_cast<SbMat*>(mat1.data()));
            mt1->matrix.setValue(m1);
            SoSeparator* sn1 = new SoSeparator();
            sn1->addChild(mt1);
            sn1->addChild(arrow);
            res->addChild(sn1);
        }
    }

    arrow->unref();
    reconstructionSep->addChild(res);
    return res;
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
            SoSeparator *n2 = new SoSeparator;
            SoMaterial* color = new SoMaterial();
            color->transparency = 0.7f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            n2->addChild(color);
            n2->addChild(n);
            objectSep->addChild(n2);
        }
    }
    if (object && object->getVisualization() && object->getVisualization()->getTriMeshModel() && UI.checkBoxNormalsObj->isChecked())
    {
        objectSep->addChild(drawNormals(object->getVisualization()->getTriMeshModel()));
    }

    reconstructionSep->removeAllChildren();
    if (reconstructedObject && UI.checkBoxReconstruction->isChecked())
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(reconstructedObject, colModel2);
        if (n)
        {
            SoSeparator *n2 = new SoSeparator;
            SoMaterial* color = new SoMaterial();
            color->transparency = 0.7f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            n2->addChild(color);
            n2->addChild(n);
            reconstructionSep->addChild(n2);
        }
    }
    if (trimesh && UI.checkBoxNormalsReconstr->isChecked())
    {
        reconstructionSep->addChild(drawNormals(trimesh));
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

    // update normalID
    for (int j = 0; j < faceCount; ++j)
    {
        MathTools::TriangleFace& face = t->faces.at(j);
        face.idNormal1 = face.id1;
        face.idNormal2 = face.id2;
        face.idNormal3 = face.id3;
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

#if 1
    PolyhedronMeshPtr m = reconstruction->reconstructMeshPoisson(points, normals);
    CGALPolyhedronMeshPtr mesh(new CGALPolyhedronMesh(m));
    trimesh = CGALMeshConverter::ConvertCGALMesh(mesh);
#else
    // does not work well...
    trimesh = reconstruction->reconstructMeshScaleSpace(points);
#endif
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
