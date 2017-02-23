
#include "SegmentedObjectViewerWindow.h"
#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/ContactConeGenerator.h"
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
#include <QFileDialog>
#include <QInputDialog>
#include <Eigen/Geometry>
#include "VirtualRobot/ManipulationObject.h"

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

#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

SegmentedObjectViewerWindow::SegmentedObjectViewerWindow(const std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->segmentedFilename = objFile;

    //boost::filesystem::path p(objFile);
    //save_dir = p.parent_path().string();

    sceneSep = new SoSeparator;
    sceneSep->ref();
    segObjectSep = new SoSeparator;
    sceneSep->addChild(segObjectSep);

    setupUI();

    loadSegmentedObject();
    buildVisu();
    viewer->viewAll();
}


SegmentedObjectViewerWindow::~SegmentedObjectViewerWindow()
{
    sceneSep->unref();
}

void SegmentedObjectViewerWindow::setupUI()
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

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonLoadSegmented, SIGNAL(clicked()), this, SLOT(reloadSegmentedObject()));
    connect(UI.comboBoxSegments, SIGNAL(currentIndexChanged(int)), this, SLOT(showSegmentedObject()));
    connect(UI.radioButtonShowSegment, SIGNAL(clicked()), this, SLOT(showSegmentedObject()));
    connect(UI.pushButtonChoseSegment, SIGNAL(clicked()), this, SLOT(choseSegments()));
    connect(UI.pushButtonScreenshot, SIGNAL(clicked()), this, SLOT(screenshot()));
}


void SegmentedObjectViewerWindow::resetSceneryAll()
{
}


void SegmentedObjectViewerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SegmentedObjectViewerWindow::buildVisu()
{
    //SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    segObjectSep->removeAllChildren();
    if (segObjectsPtr)
    {
     /*   if (segObjectsPtr->getMemberforGraspPlanning() != -1 && segObjectsPtr->members.size() > segObjectsPtr->getMemberforGraspPlanning())
        {
            stringstream s;
            s << "Grasp Planning: " << segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->name;
            UI.labelGraspPlanning->setText(s.str().c_str());
        }


        SceneObject::VisualizationType colModel1 = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;
        if (UI.radioButtonShowSegment->isChecked())
        {
            int number = UI.comboBoxSegments->currentIndex();
            stringstream ss;

            if (segObjectsPtr->members.size() < number)
            {
                cout << "Problem in Building the Visualisation" << endl;
                return;
            }

            ss << "Cluster: " << segObjectsPtr->members.at(number)->clusterNumber;
            UI.labelOut->setText(ss.str().c_str());

            for (int i = 0; i < segObjectsPtr->members.size(); i++)
            {
                if (i == number)
                {
                    VirtualRobot::ColorMap color = VirtualRobot::ColorMap::eHot;
                    VisualizationFactory::Color drawingColor;
                    if (color.getColor(i / (float)this->segObjectsPtr->members.size(), drawingColor))
                    {

                    }

                    else
                    {
                        drawingColor = VisualizationFactory::Color::Gray();
                    }
                    segObjectsPtr->members.at(i)->part->getCollisionModel()->getTriMeshModel()->setColor(drawingColor);    //overwrite color!!
                    SoNode* n = CoinVisualizationFactory::getCoinVisualization(segObjectsPtr->members.at(i)->part->getCollisionModel()->getTriMeshModel(), false, drawingColor, true);
                    //SoNode* n=CoinVisualizationFactory::getCoinVisualization(segObjectPtr->at(i).part,colModel1);
                    if (n)
                    {
                        segObjectSep->addChild(n);
                    }
                    ss << segObjectsPtr->members.at(i)->name;
                }
            }

            //maybe color corresponding to
        }
        else if (UI.radioButtonShowClusters->isChecked())
        {
            stringstream ss;
            ss << "Segments: ";
            int cluster = UI.comboBoxCluster->currentIndex();
            for (int i = 0; i < segObjectsPtr->members.size(); i++)
            {
                if (segObjectsPtr->members.at(i)->clusterNumber >= cluster)
                {
                    VirtualRobot::ColorMap color = VirtualRobot::ColorMap::eHot;
                    VisualizationFactory::Color drawingColor;
                    if (color.getColor(i / (float)this->segObjectsPtr->members.size(), drawingColor))
                        //if(color.getColor(segObjectsPtr->members.at(i).clusterNumber/(float)5.0f,drawingColor))
                    {
                    }
                    else
                    {
                        drawingColor = VisualizationFactory::Color::Gray();
                    }
                    segObjectsPtr->members.at(i)->part->getCollisionModel()->getTriMeshModel()->setColor(drawingColor);    //overwrite color!!
                    SoNode* n = CoinVisualizationFactory::getCoinVisualization(segObjectsPtr->members.at(i)->part->getCollisionModel()->getTriMeshModel(), false, drawingColor, true);
                    //SoNode* n=CoinVisualizationFactory::getCoinVisualization(segObjectPtr->at(i).part,colModel1);
                    if (n)
                    {
                        segObjectSep->addChild(n);
                    }
                    ss << segObjectsPtr->members.at(i)->name;
                }
            }
            UI.labelOut->setText(ss.str().c_str());

        }
*/
    }
    viewer->scheduleRedraw();
}

int SegmentedObjectViewerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void SegmentedObjectViewerWindow::quit()
{
    std::cout << "SegmentedObjectViewerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void SegmentedObjectViewerWindow::loadSegmentedObject()
{
/*
    std::string objectName;
    if (!segmentedFilename.empty())
    {
        boost::filesystem::path completePath(segmentedFilename);
        boost::filesystem::path branch = completePath.branch_path();
        segObjectsPtr = SegmentedObjectIO::loadManipulationObject(segmentedFilename, branch.string());
    }
    segObjectsPtr->setMemberforGraspPlanning(-1);


    if (segObjectsPtr->members.empty())
    {
        return;
    }
    //build combo box
    UI.comboBoxSegments->clear();
    for (int i = 0; i < segObjectsPtr->members.size(); i++)
    {
        UI.comboBoxSegments->addItem(QString::fromStdString((segObjectsPtr->members.at(i)->name)));
    }
    unsigned int clusters = segObjectsPtr->getNumberofCluster();
    UI.comboBoxCluster->clear();
    for (int i = 0; i < clusters; i++)
    {
        UI.comboBoxCluster->addItem(QString::number(i));
    }
*/
    buildVisu();
    viewer->viewAll();
}

void SegmentedObjectViewerWindow::showSegmentedObject()
{
    buildVisu();
}



void SegmentedObjectViewerWindow::colModel()
{
    buildVisu();
}



void SegmentedObjectViewerWindow::choseSegments()
{
    /*if (!segObjectsPtr)
    {
        cout << "First load object";
        return;
    }
    std::string text;
    bool ok;
    QString textInput = QInputDialog::getText(this, tr("Determine Segments"),
                        tr("Segments, when using more than one segment use space bar to seperate"),
                        QLineEdit::Normal, tr("seg0"), &ok);
    if (ok && !textInput.isEmpty())
    {
        text = textInput.toStdString();
    }
    else
    {
        return;
    }

    if (segObjectsPtr->members.size() >= segObjectsPtr->getMemberforGraspPlanning() && segObjectsPtr->getMemberforGraspPlanning() != -1 && grasps)
    {
        writeGraspSetToCOMSubpartPose(grasps, *segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning()));
        if (segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->part->hasGraspSet(grasps->getRobotType(), grasps->getEndEffector()))
        {
            segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->part->includeGraspSet(grasps);
        }
        else
        {
            segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->part->addGraspSet(grasps);
        }
    }
    graspsSep->removeAllChildren();
    //grasps->removeAllGrasps(); SIGABRT

    SegmentedObjectPtr segBuffer(new SegmentedObject());
    SubpartPtr newSubpart(new Subpart());
    //std::string name = "Grasp Planner - ";
    //name += eef->getName();
    newSubpart->segmentNumber = 0;
    newSubpart->clusterNumber = 0;
    newSubpart->minDiameter = 1000;
    newSubpart->maxDiameter = 0;
    newSubpart->averageDiameter = 0;
    newSubpart->numberFaces = 0;
    newSubpart->taskLabel.clear();
    //copy member variables
    segBuffer->setObjectName(segObjectsPtr->getObjectName());
    segBuffer->setNumberofCluster(segObjectsPtr->getNumberofCluster());
    segBuffer->setBoundingBox(segObjectsPtr->getBoundingBox());
    segBuffer->setCOMObject(segObjectsPtr->getCOMObject());
    segBuffer->setSegmentationTime((segObjectsPtr->getSegmentationTime()));
    segBuffer->setTotalTime(segObjectsPtr->getTotalTime());
    segBuffer->setNrFaces(segObjectsPtr->getNrFaces());
    //add more class variable when you add more


    //reunite segmentated objects back
    std::vector<std::string> segmentsNames;


    split(text, segmentsNames);
    //cout<<segmentsNames.size();

    //check if all segmentsNames exists
    for (int i = 0; i < segmentsNames.size(); i++)
    {
        bool segmentNameExists = false;
        for (int j = 0; j < segObjectsPtr->members.size(); j++)
        {
            if (segmentsNames.at(i) == segObjectsPtr->members.at(j)->name)
            {
                segmentNameExists = true;
            }
        }
        if (!segmentNameExists)
        {
            GRASPSTUDIO_WARNING << "The Segment Name " << segmentsNames.at(i) << " does not exist";
            return;
        }
    }


    if (segmentsNames.empty())
    {
        cout << "SegmentsNames are empty" << endl;
        return;
    }


    std::string nameforGraspPlanning;
    if (!segmentsNames.empty())
    {
        nameforGraspPlanning = segmentsNames.at(0);
    }
    //create name for grasp planning
    for (int i = 1; i < segmentsNames.size(); i++)
    {
        nameforGraspPlanning += "&";
        nameforGraspPlanning += segmentsNames.at(i);
    }
    //segBuffer.setNameforGraspPlanning(nameforGraspPlanning);
    newSubpart->name = nameforGraspPlanning;

    //reunite the segments in segmentsNames

    TriMeshModelPtr bufferPtr(new TriMeshModel());
    TriMeshModelPtr wholeObjectTriMesh(new TriMeshModel());


    //for all segmentsNames add all Triangles to triMeshGraspPlanning
    for (int j = 0; j < segObjectsPtr->members.size(); j++)
    {
        bool isinSegmentedNames = false;
        for (int i = 0; i < segmentsNames.size(); i++)
        {
            if (segmentsNames.at(i) == segObjectsPtr->members.at(j)->name)
            {
                //member with j should be added to the object
                ManipulationObjectPtr objectToBeAdded = segObjectsPtr->members.at(j)->part;
                addToTriMeshModel(bufferPtr, objectToBeAdded->getVisualization()->getTriMeshModel());
                //add grasp sets
                //needed for class variables of Subpart
                newSubpart->numberFaces += segObjectsPtr->members.at(j)->numberFaces;
                newSubpart->averageDiameter += segObjectsPtr->members.at(j)->averageDiameter * segObjectsPtr->members.at(j)->numberFaces;
                newSubpart->segmentNumber = segObjectsPtr->members.at(j)->segmentNumber;      //is last segment number bad for visualization
                if (segObjectsPtr->members.at(j)->minDiameter <= newSubpart->minDiameter)
                {
                    newSubpart->minDiameter = segObjectsPtr->members.at(j)->minDiameter;
                }
                if (segObjectsPtr->members.at(j)->maxDiameter >= newSubpart->maxDiameter)
                {
                    newSubpart->maxDiameter = segObjectsPtr->members.at(j)->maxDiameter;
                }
                if (segObjectsPtr->members.at(j)->clusterNumber >= newSubpart->clusterNumber)
                {
                    newSubpart->clusterNumber = segObjectsPtr->members.at(j)->clusterNumber;
                }
                isinSegmentedNames = true;

                //add task label - add always just first
                if (newSubpart->taskLabel.empty())
                {
                    newSubpart->taskLabel = segObjectsPtr->members.at(j)->taskLabel;
                }
            }
            //create for whole object tri mesh model
            addToTriMeshModel(wholeObjectTriMesh, segObjectsPtr->members.at(j)->part->getVisualization()->getTriMeshModel());
        }
        if (!isinSegmentedNames)
        {
            segBuffer->members.push_back(segObjectsPtr->members.at(j));
        }
    }




    newSubpart->averageDiameter /= newSubpart->numberFaces;
    newSubpart->COM = Eigen::Matrix4f::Identity();
    newSubpart->COM.block(0, 3, 3, 1) = bufferPtr->getCOM();
    segBuffer->setMemberforGraspPlanning(segBuffer->members.size());

    //create new manipulation object
    CoinVisualizationFactory factory;

    Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
    VisualizationNodePtr visu = factory.createTriMeshModelVisualization(bufferPtr, false, gp);
    VisualizationNodePtr visuWhole = factory.createTriMeshModelVisualization(wholeObjectTriMesh, false, gp);
    std::string filenameWRL = save_dir + "/wrl/" + nameforGraspPlanning + ".wrl";
    visu->setFilename(filenameWRL, false); //was macht boundingbox true/ false?

    std::string colModelName = newSubpart->name;
    colModelName += "_VISU_ColModel";
    CollisionModelPtr collisionModel(new CollisionModel(visu, colModelName, CollisionCheckerPtr()));
    CollisionModelPtr collisionModelWhole(new CollisionModel(visuWhole, segObjectsPtr->getObjectName(), CollisionCheckerPtr()));

    wholeObject.reset(new ManipulationObject(segObjectsPtr->getObjectName(), visuWhole, collisionModelWhole));
    newSubpart->part.reset(new ManipulationObject(newSubpart->name, visu, collisionModel));

    //add the old grasp to the new object

    for (int j = 0; j < segObjectsPtr->members.size(); j++)
    {
        for (int i = 0; i < segmentsNames.size(); i++)
        {
            if (segmentsNames.at(i) == segObjectsPtr->members.at(j)->name)
            {
                //writeGraspSetToObjectPose();
                addAllGrasps(segObjectsPtr->members.at(j), newSubpart);
            }
        }
    }
    segBuffer->members.push_back(newSubpart);
    segObjectsPtr.reset(new SegmentedObject());
    segObjectsPtr = segBuffer;
    //count ==1?

    UI.comboBoxSegments->clear();
    for (int i = 0; i < segObjectsPtr->members.size(); i++)
    {
        UI.comboBoxSegments->addItem(QString::fromStdString((segObjectsPtr->members.at(i)->name)));
    }

    determinePowerOrPrecisionGrasp();

    if (segObjectsPtr->members.size() < segObjectsPtr->getMemberforGraspPlanning() || segObjectsPtr->getMemberforGraspPlanning() == -1)
    {
        cout << "The size of members in segObjectsPtr is smaller than getMemberforGraspPlanning" << endl;
        return;
    }
    // extend maybe -> to all subparts
    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->part));
    qualityMeasure->calculateObjectProperties();
    //copy ows min offset and volume to subpart
    segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->OWSMinOffset = qualityMeasure->getOWSMinOffset();
    segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->OWSVolume = qualityMeasure->getOWSVolume();
    segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->OWSCalculated = (int)true;



    approach.reset(new GraspStudio::ApproachMovementSurfaceNormalSegmented(segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->part, eef));


    approach->setAverageDiameter(segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning())->averageDiameter);
    eefCloned = approach->getEEFRobotClone();

    if (robot && eef)
    {
        std::string name = "Grasp Planner - ";
        name += eef->getName();
        grasps.reset(new GraspSet(name, robot->getType(), eefName));
    }

    planner.reset(new GraspStudio::GenericGraspPlanner(grasps, qualityMeasure, approach));
    planner->setVerbose(true);

    //cout << "Do Grasp Planning on Segments" << segObjectsPtr->members.at(segObjectsPtr->getMemberforGraspPlanning()).name<<endl;

    contacts.clear(); //hinder the visu friction cones
    buildVisu();
    */
}

void SegmentedObjectViewerWindow::reloadSegmentedObject()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Segmented Object"), QString(), tr("XML Files (*.xml)"));
    segmentedFilename = std::string(fi.toAscii());
    if (segmentedFilename.empty())
    {
        return;
    }

    //boost::filesystem::path p(segmentedFilename);
    //save_dir = p.parent_path().string();

    loadSegmentedObject();
}

void SegmentedObjectViewerWindow::screenshot()
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

