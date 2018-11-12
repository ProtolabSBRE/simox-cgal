
#include "ObjectSegmentationSkeletonWindow.h"
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/XML/rapidxml.hpp>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>
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
#include <Inventor/nodes/SoAnnotation.h>

#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include "Segmentation/Skeleton/SkeletonPart.h"
#include "Segmentation/Skeleton/MeshSkeletonData.h"
#include "IO/SkeletonIO.h"
#include "IO/CGALMeshIO.h"
//#include "ObjectSegmentationSkeletonWindowIO.h"

#include <QImage>
#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;

//float TIMER_MS = 30.0f;

ObjectSegmentationSkeletonWindow::ObjectSegmentationSkeletonWindow(const std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start with " << objFile << endl;

    this->objectFilename = objFile;

    sceneSep = new SoSeparator;
    sceneSep->ref();

    objectSep = new SoSeparator;
    sceneSep->addChild(objectSep);

    skeletonSep = new SoSeparator;
    sceneSep->addChild(skeletonSep);

    segmentationSep = new SoSeparator;
    //SoAnnotation* an = new SoAnnotation;
    //an->addChild(segmentationSep);
    //sceneSep->addChild(an);
    sceneSep->addChild(segmentationSep);

    surfaceSep = new SoSeparator;
    sceneSep->addChild(surfaceSep);


    setupUI();

    loadObject();
    buildVisu();
    viewer->viewAll();
}


ObjectSegmentationSkeletonWindow::~ObjectSegmentationSkeletonWindow()
{
//    sceneSep->unref();
}

void ObjectSegmentationSkeletonWindow::setupUI()
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

    //connect(UI.checkBoxManip, SIGNAL(clicked()), this, SLOT(buildVisu()));
    //connect(UI.checkBoxSkeleton, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxLines, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.horizontalSliderTr, SIGNAL(valueChanged(int)), this, SLOT(buildVisu()));
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

    connect(UI.radioButtonObjectNo, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonObjectOrig, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonObjectSeg, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonSkelNo, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonSkel, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonSkelSeg, SIGNAL(clicked()), this, SLOT(buildVisu()));

}


void ObjectSegmentationSkeletonWindow::resetSceneryAll()
{
}


void ObjectSegmentationSkeletonWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void ObjectSegmentationSkeletonWindow::buildVisu()
{
    SceneObject::VisualizationType colModel = (UI.radioButtonColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    ///////////// OBJECT
    objectSep->removeAllChildren();
    if (manipObject && UI.radioButtonObjectOrig->isChecked())
    {
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(manipObject, colModel);
        //visualizationObject = manipObject->getVisualization<CoinVisualization>();
        //visualizationObject->setTransparency(0.7f);
        //SoNode* n = visualizationObject->getCoinVisualization();

        if (n)
        {
            SoMaterial* color = new SoMaterial();
            //color->transparency = 0.7f;
            color->transparency = UI.horizontalSliderTr->value()/100.0f;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
            objectSep->addChild(color);

            objectSep->addChild(n);
        }

    }



    ////////////////// SKELETON
    skeletonSep->removeAllChildren();
    if (skeleton && UI.radioButtonSkel->isChecked())
    {
        SoSeparator* s = new SoSeparator();
        s->addChild(CGALCoinVisualization::CreateSkeletonVisualization(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.checkBoxLines->isChecked()));
        skeletonSep->addChild(s);

        if (UI.checkBoxSkeletonPoint->isChecked())
        {
            skeletonSep->addChild(CGALCoinVisualization::ShowSkeletonPoint(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.spinBoxSkeletonPoint->value()));
        }

    }



    /////////////////// SEGMENTATION (SKELETON)
    segmentationSep->removeAllChildren();

    bool lines = UI.checkBoxLines->isChecked();
    //bool pigment = UI.checkBoxSegment->isChecked();

    SoMaterial* partColor = new SoMaterial;
    partColor->diffuseColor.setValue(1.f, 0.f, 0.f);

    if (skeleton && segSkeleton && UI.radioButtonSkelSeg->isChecked())
    {
        std::vector<ObjectPartPtr> members;
        if (segSkeleton->getSegmentedObject())
            members = segSkeleton->getSegmentedObject()->getObjectParts();

        SoSeparator* s2 = new SoSeparator;
        segmentationSep->addChild(s2);


        SkeletonPtr s = skeleton->getSkeleton();

        /*if (colorizeOneSegment)
        {
            s2->addChild(partColor);
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(index_segmentation));
            SoSeparator* segment = CGALCoinVisualization::CreateSegmentVisualization(s, surfaceMesh->getMesh(), subpart, lines);
            s2->addChild(segment);

        } else if (colorizeAllSegments)
        {*/

            SoSeparator* all = CGALCoinVisualization::CreateSegmentationVisualization(s, surfaceMesh->getMesh(), members, lines, 0.8f, 13.0f);
            s2->addChild(all);
        //}

        if (UI.checkBoxSkeletonPoint->isChecked())
        {
            s2->addChild(CGALCoinVisualization::ShowSkeletonPoint(skeleton->getSkeleton(), surfaceMesh->getMesh(), UI.spinBoxSkeletonPoint->value()));
        }
    }

    /////////////////// SEGMENTATION (SURFACE)

    surfaceSep->removeAllChildren();
    if (UI.radioButtonObjectSeg->isChecked() && skeleton && segSkeleton)
    {
        //int number_segmentation = UI.comboBoxSegmentation->count();
        int index_segmentation = UI.comboBoxSegmentation->currentIndex();
        std::vector<ObjectPartPtr> members;
        if (segSkeleton->getSegmentedObject())
            members = segSkeleton->getSegmentedObject()->getObjectParts();
        bool colorizeOneSegment = (size_t(index_segmentation) < members.size());
        bool colorizeAllSegments = (size_t(index_segmentation) == members.size());

        SkeletonPtr s = skeleton->getSkeleton();

        if (colorizeOneSegment)
        {
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(index_segmentation));
            SoNode* segment = CGALCoinVisualization::CreatePigmentedSubpartVisualization(s, surfaceMesh->getMesh(), subpart);
            surfaceSep->addChild(segment);

        } else if (colorizeAllSegments)
        {

            SoSeparator* all = CGALCoinVisualization::CreatePigmentedMeshVisualization(s, surfaceMesh->getMesh(), members, members.size());
            surfaceSep->addChild(all);
        }

    }

    viewer->scheduleRedraw();
}

int ObjectSegmentationSkeletonWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void ObjectSegmentationSkeletonWindow::quit()
{
    std::cout << "ObjectSegmentationSkeleton: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void ObjectSegmentationSkeletonWindow::saveSegmentedObject()
{
    if (!skeleton || !segSkeleton || !surfaceMesh)
    {
        VR_INFO << "Data not created. Press button 'segmentation' first." << endl;
        return;
    }

    boost::filesystem::path segmentedObjectFileDefaultName (objectFilename);

    segmentedObjectFileDefaultName.replace_extension(".soxml");
    cout << "manipObjectFile: " << objectFilename << endl;

    //QString fi = QFileDialog::getSaveFileName(this, tr("Save Segmented Object"), QString(segmentedObjectFileDefaultName.c_str()), tr("Segmented Object XML Files (*.soxml)"));
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    QStringList nameFilters;
    nameFilters << "Segmented Object XML Files (*.soxml)"
                << "All Files (*.*)";
    dialog.setNameFilters(nameFilters);

    if (dialog.exec())
    {
        if (dialog.selectedFiles().size() == 0)
            return;

        fi = dialog.selectedFiles()[0];
    }
    else
    {
        VR_INFO << "save dialog canceled" << std::endl;
        return;
    }

    if(fi.isEmpty())
        return;
    std::string segObjectFile = std::string(fi.toLatin1());

    boost::filesystem::path segObjectFilePath(segObjectFile);
    if(segObjectFilePath.extension().empty())
    {
        segObjectFilePath.replace_extension(".soxml");
    }

    bool save = MeshSkeletonData::saveSkeletonData(segObjectFilePath.string(), objectFilename, skeleton, surfaceMesh, segSkeleton->getSegmentedObject());

    if (!save)
    {
        return;
    }
}


void ObjectSegmentationSkeletonWindow::loadData()
{

    VR_INFO << "Loading skeleton ...\n";

    //QString fi = QFileDialog::getOpenFileName(this, tr("Open Skeleton File"), QString(), tr("XML Files (*.xml)"));
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    QStringList nameFilters;
    nameFilters << "XML Files (*.xml)"
                << "Segemented Object Files (*.soxml)"
                << "All Files (*.*)";
    dialog.setNameFilters(nameFilters);

    if (dialog.exec())
    {
        if (dialog.selectedFiles().size() == 0)
            return;

        fi = dialog.selectedFiles()[0];
    }
    else
    {
        VR_INFO << "load dialog canceled" << std::endl;
        return;
    }

    string file(fi.toLatin1());

    VR_INFO << "Loading from file: " << file << endl;
    if (file.empty())
        return;

    try {
        MeshSkeletonDataPtr data = MeshSkeletonData::loadSkeletonData(file);
        if (data)
        {
            manipObject = data->manipObject;
            objectFilename = manipObject->getFilename();
            surfaceMesh = data->surfaceMesh;
            skeleton = data->skeleton;
            segSkeleton = data->segSkeleton;
        }
    } catch(rapidxml::parse_error& e)
    {
        VR_ERROR << "Could not parse data in xml definition" << endl
                           << "Error message:" << e.what() << endl
                           << "Position: " << endl << e.where<char>() << endl;
        return;
    }
    catch (VirtualRobotException& e)
    {
        VR_ERROR << " ERROR while loading object" << endl;
        VR_ERROR << e.what();
        return;
    }
    catch (...)
    {
        VR_ERROR << " ERROR while loading object" << endl;
        return;
    }

    UI.comboBoxSegmentation->clear();

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

void ObjectSegmentationSkeletonWindow::loadObject()
{
    try {
        manipObject = ObjectIO::loadManipulationObject(objectFilename);
    } catch(...)
    {
        VR_ERROR << "Could not load manipulation object from " << objectFilename << endl;
        manipObject.reset();
    }

    buildVisu();
    viewer->viewAll();
}


void ObjectSegmentationSkeletonWindow::colModel()
{
    buildVisu();
}


void ObjectSegmentationSkeletonWindow::reloadObject()
{
    segSkeleton.reset();
    surfaceMesh.reset();
    skeleton.reset();

    UI.comboBoxSegmentation->clear();

    //QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("XML Files (*.xml)"));
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    QStringList nameFilters;
    nameFilters << "Manipulation Files (*.moxml)"
                << "XML Files (*.xml)"
                << "All Files (*.*)";
    dialog.setNameFilters(nameFilters);

    if (dialog.exec())
    {
        if (dialog.selectedFiles().size() == 0)
            return;

        fi = dialog.selectedFiles()[0];
    }
    else
    {
        VR_INFO << "load dialog canceled" << std::endl;
        return;
    }

    objectFilename = std::string(fi.toLatin1());
    if (objectFilename.empty())
        return;

    loadObject();
}


void ObjectSegmentationSkeletonWindow::buildObject()
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


     if (UI.radioButtonR5->isChecked())
     {
         VR_INFO << "Remeshing ..." << endl;
         VirtualRobot::ObstaclePtr p = GraspStudio::MeshConverter::RefineObjectSurface(manipObject, 5.f);
         model = p->getCollisionModel()->getTriMeshModel();
         model->mergeVertices();
         VR_INFO << "Remeshing done.Vertices: " << model->vertices.size() << " and faces: " << model->faces.size() << endl;
     } else if (UI.radioButtonR10->isChecked())
     {
         VR_INFO << "Remeshing ..." << endl;
         VirtualRobot::ObstaclePtr p = GraspStudio::MeshConverter::RefineObjectSurface(manipObject, 10.f);
         model = p->getCollisionModel()->getTriMeshModel();
         model->mergeVertices();
         VR_INFO << "Remeshing done.Vertices: " << model->vertices.size() << " and faces: " << model->faces.size() << endl;
     } else
         VR_INFO << "Using original model.Vertices: " << model->vertices.size() << " and faces: " << model->faces.size() << endl;

     VR_INFO << "Converting mesh to cgal structure..." << endl;

     surfaceMesh = CGALMeshConverter::ConvertToSurfaceMesh(model);

     if (!CGAL::is_triangle_mesh(*(surfaceMesh->getMesh())))
     {
         VR_WARNING << "Not a triangle mesh!" << endl;
     }
     if (!CGAL::is_closed(*(surfaceMesh->getMesh())))
     {
         VR_WARNING << "Mesh not closed!" << endl;
     }

    VR_INFO << "Calculating skeleton ..." << endl;

    skeleton = CGALSkeletonPtr(new CGALSkeleton(manipObject->getName(), surfaceMesh->getMesh()));
    skeleton->initParameters();
    skeleton->calculateSkeleton();

    VR_INFO << "Done in " << skeleton->getTime() << " ms " << endl;

    float distBranch = UI.dsbDistBranch->value();

    VR_INFO << "Calculating skeleton segmentation ..." << endl;

    segSkeleton = MeshSkeletonPtr(new MeshSkeleton(surfaceMesh, skeleton->getSkeleton(), distBranch));

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

void ObjectSegmentationSkeletonWindow::renderDepthSkeletonImage()
{
    VirtualRobot::RobotPtr robot;
    std::string robotFilename("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFilename);

    QFileInfo fileInfo(robotFilename.c_str());
    std::string suffix(fileInfo.suffix().toLatin1());
    RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

    if (!importer)
    {
        cout << " ERROR while grabbing importer" << endl;
        return;
    }
    const int width = 640;
    const int height = 480;
    robot = importer->loadFromFile(robotFilename, RobotIO::eStructure);
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,1>(0,3) =  Eigen::Vector3f(0, -500, -1700);
    robot->setGlobalPose(pose);
    auto cam = robot->getRobotNode("EyeLeftCamera");

    auto rendererCam = CoinVisualizationFactory::createOffscreenRenderer(width, height);


    auto saveDepthImage = [&](QString& path, SoSeparator* separator)
    {
        std::vector<unsigned char> rgbImage, greyscale;
        std::vector<float> depthImage;
        std::vector<Eigen::Vector3f> pointcloud;
        VirtualRobot::CoinVisualizationFactory::renderOffscreenRgbDepthPointcloud(rendererCam, cam, separator,
                                                                                  width, height, true, rgbImage, true, depthImage, false, pointcloud);
        float maxZCut = 700;
        greyscale.resize(height*width*3);
        for(std::size_t index = 0; index < static_cast<std::size_t>(width*height); ++index)
        {
            const float distance = depthImage.at(index);
            const unsigned char value = (distance>=maxZCut)?255:distance/maxZCut*255.f;

            greyscale.at(3 * index    ) = value;
            greyscale.at(3 * index + 1) = value;
            greyscale.at(3 * index + 2) = value;
        }


        QImage i(greyscale.data(), width, height, QImage::Format_RGB888);

        bool bRes = i.save(path, "PNG");
        if (bRes)
        {
            cout << "wrote image " << path.toStdString() << endl;
        }
        else
        {
            cout << "failed writing image " << path.toStdString() << endl;
        }
    };

    QString path = "/tmp/object.png";
    QString pathSkeleton = "/tmp/skeleton.png";
    saveDepthImage(path, objectSep);
    saveDepthImage(pathSkeleton, skeletonSep);

    delete rendererCam;

}

void ObjectSegmentationSkeletonWindow::screenshot()
{
    //SoCamera* camera = viewer->getCamera();
    //Fit to object representation
    //camera->orientation.setValue(SbVec3f(0,1,1),1.5707963f); //hammer
    //camera->orientation.setValue(SbVec3f(0,0,1),0.7853981f); //teddy
    //camera->orientation.setValue(SbVec3f(0,1,0),1.57079f); //screwdriver
    //viewer->viewAll();

    QString fi = QFileDialog::getSaveFileName(this, tr("Save Screenshot"));
    std::string buffer = std::string(fi.toLatin1());
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

