
#include "SkeletonViewerWindow.h"
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/rapidxml.hpp>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <GraspPlanning/MeshConverter.h>
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

#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include "Segmentation/Skeleton/SkeletonPart.h"
#include "Segmentation/Skeleton/MeshSkeletonData.h"
#include "IO/SkeletonIO.h"
#include "IO/CGALMeshIO.h"
//#include "SkeletonViewerWindowIO.h"

#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;

//float TIMER_MS = 30.0f;

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
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadData()));
    connect(UI.pushButtonScreenshot, SIGNAL(clicked()), this, SLOT(screenshot()));
    connect(UI.comboBoxSegmentation, SIGNAL(currentIndexChanged(int)), this, SLOT(buildVisu()));
    connect(UI.spinBoxSkeletonPoint, SIGNAL(valueChanged(int)), this, SLOT(buildVisu()));
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
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(manipObject, colModel);
        //visualizationObject = manipObject->getVisualization<CoinVisualization>();
        //visualizationObject->setTransparency(0.7f);
        //SoNode* n = visualizationObject->getCoinVisualization();

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

    skeletonSep->removeAllChildren();

    if (skeleton && UI.radioButtonFullModel->isChecked())
    {
        if (UI.checkBoxSkeleton->isChecked())
        {
            SoSeparator* s = new SoSeparator();
            SoMaterial* color = new SoMaterial();
            color->diffuseColor.setValue(1.f, 0.f, 0.f);
            s->addChild(color);
            s->addChild(CGALCoinVisualization::CreateSkeletonVisualization(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.checkBoxLines->isChecked()));
            skeletonSep->addChild(s);
        }


        if (UI.checkBoxSkeletonPoint->isChecked())
        {
            skeletonSep->addChild(CGALCoinVisualization::ShowSkeletonPoint(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.spinBoxSkeletonPoint->value()));
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

        if (size_t(index_segmentation) < members.size())
        {
            segmentationSep->addChild(partColor);
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(index_segmentation));
            SoSeparator* segment = CGALCoinVisualization::CreateSegmentVisualization(s, surfaceMesh->getMesh(), subpart, lines);
            segmentationSep->addChild(segment);

        } else if (size_t(index_segmentation) == members.size()){

            SoSeparator* all = CGALCoinVisualization::CreateSegmentationVisualization(s, surfaceMesh->getMesh(), members, lines);
            segmentationSep->addChild(all);
        }
    }

    surfaceSep->removeAllChildren();
    if (pigment && skeleton)
    {
        SkeletonPtr s = skeleton->getSkeleton();
        std::vector<ObjectPartPtr> members = segSkeleton->getSegmentedObject()->getObjectParts();

        if (size_t(index_segmentation) < members.size())
        {
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(index_segmentation));
            SoNode* segment = CGALCoinVisualization::CreatePigmentedSubpartVisualization(s, surfaceMesh->getMesh(), subpart, VirtualRobot::VisualizationFactory::Color(1.f, 0.f, 0.f));
            surfaceSep->addChild(segment);

        } else if (size_t(index_segmentation) == members.size()){

            SoSeparator* all = CGALCoinVisualization::CreatePigmentedMeshVisualization(s, surfaceMesh->getMesh(), members, members.size());
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
    if (!skeleton || !segSkeleton || !surfaceMesh)
    {
        VR_INFO << "Data not created. Press button 'segmentation' first." << endl;
        return;
    }

    string object_dir = objectFilename;

    cout << "objectFile: " << object_dir << endl;

    QString fi = QFileDialog::getSaveFileName(this, tr("Save SegmentedObject"), QString(objectFilename.c_str()), tr("XML Files (*.xml)"));
    objectFile = std::string(fi.toLatin1());

    bool save = MeshSkeletonData::saveSkeletonData(objectFile, object_dir, skeleton, surfaceMesh, segSkeleton->getSegmentedObject());

    if (!save)
    {
        return;
    }
}


void SkeletonViewerWindow::loadData()
{

    VR_INFO << "Loading skeleton ...\n";

    QString fi = QFileDialog::getOpenFileName(this, tr("Open Skeleton File"), QString(), tr("XML Files (*.xml)"));
    string file(fi.toLatin1());

    cout << "file: " << file << endl;

    try {
        MeshSkeletonDataPtr data = MeshSkeletonData::loadSkeletonData(file);
        if (data)
        {
            manipObject = data->manipObject;
            surfaceMesh = data->surfaceMesh;
            skeleton = data->skeleton;
            segSkeleton = data->segSkeleton;
        }
    } catch(rapidxml::parse_error& e)
    {
        THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                           << "Error message:" << e.what() << endl
                           << "Position: " << endl << e.where<char>() << endl);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while loading object" << endl;
        cout << e.what();
        return;
    }



    vector<ObjectPartPtr> seg = segSkeleton->getSegmentedObject()->getObjectParts();
    for (size_t i = 0; i < segSkeleton->getSegmentedObject()->getObjectParts().size(); i++)
    {

        SkeletonPartPtr tmp = boost::static_pointer_cast<SkeletonPart>(seg.at(i));
        string s = tmp->name;
        UI.comboBoxSegmentation->addItem(QString(s.c_str()));
    }

    UI.comboBoxSegmentation->addItem(QString("All segments"));
    UI.comboBoxSegmentation->addItem(QString("No segment"));

    UI.comboBoxSegmentation->setCurrentIndex(segSkeleton->getSegmentedObject()->getObjectParts().size() + 1);


    VR_INFO << "Loading complete.\n";

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

    VirtualRobot::TriMeshModelPtr model = manipObject->getCollisionModel()->getTriMeshModel();

     VR_INFO << "Remeshing ..." << endl;

     if (UI.radioButtonR5->isChecked())
     {
         VirtualRobot::ObstaclePtr p = GraspStudio::MeshConverter::RefineObjectSurface(manipObject, 5.f);
         model = p->getCollisionModel()->getTriMeshModel();

     } else if (UI.radioButtonR10->isChecked())
     {
         VirtualRobot::ObstaclePtr p = GraspStudio::MeshConverter::RefineObjectSurface(manipObject, 10.f);
         model = p->getCollisionModel()->getTriMeshModel();

     }

     VR_INFO << "Remeshing done.Vertices: " << model->vertices.size() << " and faces: " << model->faces.size() << endl;



    VR_INFO << "Converting mesh to cgal structure..." << endl;

    surfaceMesh = CGALMeshConverter::ConvertToSurfaceMesh(model);

    VR_INFO << "Calculatin skeleton ..." << endl;

    skeleton = CGALSkeletonPtr(new CGALSkeleton(manipObject->getName(), surfaceMesh->getMesh()));
    skeleton->initParameters();
    skeleton->calculateSkeleton();

    VR_INFO << "Done in " << skeleton->getTime() << " ms " << endl;


    VR_INFO << "Calculatin skeleton segmentation ..." << endl;

    segSkeleton = MeshSkeletonPtr(new MeshSkeleton(surfaceMesh, skeleton->getSkeleton(), 20.0));

    VR_INFO << "Done in " << segSkeleton->getTime() << " ms " << endl;


    UI.comboBoxSegmentation->clear();
    vector<ObjectPartPtr> seg = segSkeleton->getSegmentedObject()->getObjectParts();
    for (size_t i = 0; i < segSkeleton->getSegmentedObject()->getObjectParts().size(); i++)
    {

        SkeletonPartPtr tmp = boost::static_pointer_cast<SkeletonPart>(seg.at(i));
        tmp->calculateLengthOfSegment(skeleton->getSkeleton());
        string s = tmp->name;
        UI.comboBoxSegmentation->addItem(QString(s.c_str()));
    }

    UI.comboBoxSegmentation->addItem(QString("All segments"));
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

