#include "SkeletonGraspPlannerWindow.h"
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

#include "GraspPlanning/Skeleton/SkeletonVertexAnalyzer.h"
#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include "Segmentation/Skeleton/MeshSkeletonData.h"

#include <sstream>


using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

SkeletonGraspPlannerWindow::SkeletonGraspPlannerWindow(std::string& robFile, std::string& eefName, std::string& preshape, std::string& segmentedObjectFile)
    : QMainWindow(NULL), skeleton(new Skeleton), segmentation(new SegmentedObject())
{
    VR_INFO << " start " << endl;

    // init the random number generator
    srand(time(NULL));

    this->robotFile = robFile;
    this->segmentedObjectFile = segmentedObjectFile;
    this->eefName = eefName;
    this->preshape = preshape;
    eefVisu = NULL;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    graspsSep = new SoSeparator;
    skeletonSep = new SoSeparator;
    graspsSep->ref();

    test = new SoSeparator;
    test->ref();

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    sceneSep->addChild(test);
    //sceneSep->addChild(graspsSep);

    setupUI();


    loadRobot();
    loadSegmentedObject(segmentedObjectFile);
//    initPlanner();
    buildVisu();
    viewer->viewAll();
}


SkeletonGraspPlannerWindow::~SkeletonGraspPlannerWindow()
{
    sceneSep->unref();
    graspsSep->unref();
    test->unref();

    if (eefVisu)
    {
        eefVisu->unref();
    }

}



/*void GraspPlannerWindow::timerCB(void * data, SoSensor * sensor)
{
    GraspPlannerWindow *ikWindow = static_cast<GraspPlannerWindow*>(data);
    float x[6];
    x[0] = (float)ikWindow->UI.horizontalSliderX->value();
    x[1] = (float)ikWindow->UI.horizontalSliderY->value();
    x[2] = (float)ikWindow->UI.horizontalSliderZ->value();
    x[3]= (float)ikWindow->UI.horizontalSliderRo->value();
    x[4] = (float)ikWindow->UI.horizontalSliderPi->value();
    x[5] = (float)ikWindow->UI.horizontalSliderYa->value();
    x[0] /= 10.0f;
    x[1] /= 10.0f;
    x[2] /= 10.0f;
    x[3] /= 300.0f;
    x[4] /= 300.0f;
    x[5] /= 300.0f;

    if (x[0]!=0 || x[1]!=0 || x[2]!=0 || x[3]!=0 || x[4]!=0 || x[5]!=0)
        ikWindow->updateObject(x);
}*/

void SkeletonGraspPlannerWindow::setupUI()
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


    connect(UI.radioButtonNothing, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonSkeleton, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonSegmentation, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxHand, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxGraspingInterval, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxGraspingPlane, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxGCP, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxVerbose, SIGNAL(clicked()), this, SLOT(setVerbose()));

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.pushButtonPlanAll, SIGNAL(clicked()), this, SLOT(planAll()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonLoadData, SIGNAL(clicked()), this, SLOT(loadData()));

    connect(UI.spinBoxGraspNumberPlanned, SIGNAL(valueChanged(int)), this, SLOT(selectGrasp()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
//    connect(UI.checkBoxGCP, SIGNAL(clicked()), this, SLOT(showGrasps()));
//    connect(UI.checkBoxPoints, SIGNAL(clicked()), this, SLOT(buildVisu()));

}

void SkeletonGraspPlannerWindow::updateSkeletonInfo()
{
    std::stringstream ss;
    ss << std::setprecision(3);
    int nrSegments = -1;
    int nrVertices = -1;
    if (segmentation)
    {
        nrSegments = int(segmentation->getObjectParts().size());
        // get vertex count
        nrVertices = 0;
        for (int i=0; i<nrSegments; i++)
        {
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(i));
            nrVertices += (int)subpart->sortedSkeletonPartIndex.size();
        }
    }
    ss << "Segments: " << nrSegments << "\nSkeleton Vertices: " << nrVertices <<"\n";
    int curSeg = -1;
    int curVert = -1;

    if (approach)
    {
        curSeg = approach->getCurrentSegment();
        curVert = approach->getCurrentVertex();
    }

    ss << "Current Segment: " << curSeg << "\nCurrent Vertex: " << curVert <<"\n";

    UI.labelSkeleton->setText(QString(ss.str().c_str()));
}


void SkeletonGraspPlannerWindow::resetSceneryAll()
{
    if (grasps)
    {
        grasps->removeAllGrasps();
    }

    graspsSep->removeAllChildren();

    initPlanner();

    updateSkeletonInfo();
}


void SkeletonGraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SkeletonGraspPlannerWindow::buildVisu()
{

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned && UI.checkBoxHand->isChecked())
    {
        // disable all other preshape GCP nodes
        for (std::string &preshapeName : eefCloned->getEndEffector(eefName)->getPreshapes())
        {
            RobotConfigPtr config = eefCloned->getEndEffector(eefName)->getPreshape(preshapeName);
            if (config->getTCP())
                config->getTCP()->showCoordinateSystem(false);
        }
        if (eefCloned->getEndEffector(eefName)->hasPreshape(preshape))
        {
            RobotConfigPtr config = eefCloned->getEndEffector(eefName)->getPreshape(preshape);
            std::string gcp = "GCP";
            if (config->getTCP())
                config->getTCP()->showCoordinateSystem(UI.checkBoxGCP->isChecked(), 0.5f, &gcp);
        }

        visualizationRobot = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            visualizationRobot->highlight(UI.checkBoxHighlight->isChecked());
        }
    }

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

        /*visualizationObject = object->getVisualization<CoinVisualization>(colModel2);
        visualizationObject->colorize(VisualizationFactory::Color::Gray());
        visualizationObject->setTransparency(0.7f);
        SoNode* visualisationNode = visualizationObject->getCoinVisualization();

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }*/
    }

    frictionConeSep->removeAllChildren();
    bool fc = (UI.checkBoxCones->isChecked());

    if (fc && contacts.size() > 0 && qualityMeasure)
    {
        ContactConeGeneratorPtr cg = qualityMeasure->getConeGenerator();
        float radius = cg->getConeRadius();
        float height = cg->getConeHeight();
        float scaling = 30.0f;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts, height * scaling, radius * scaling, true);

        if (visualisationNode)
        {
            frictionConeSep->addChild(visualisationNode);
        }

        // add approach dir visu
        for (size_t i = 0; i < contacts.size(); i++)
        {
            SoSeparator* s = new SoSeparator;
            Eigen::Matrix4f ma;
            ma.setIdentity();
            ma.block(0, 3, 3, 1) = contacts[i].contactPointFingerGlobal;
            SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransformScaleMM2M(ma);
            s->addChild(m);
            s->addChild(CoinVisualizationFactory::CreateArrow(contacts[i].approachDirectionGlobal, 10.0f, 1.0f));
            frictionConeSep->addChild(s);
        }
    }

    if (UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) < 0)
    {
        sceneSep->addChild(graspsSep);
    }

    if (!UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) >= 0)
    {
        sceneSep->removeChild(graspsSep);
    }


    if (UI.groupBoxSkeleton->isEnabled())
    {
        skeletonSep->removeAllChildren();
        sceneSep->addChild(skeletonSep);

        vector<ObjectPartPtr> members = segmentation->getObjectParts();

        if(UI.radioButtonSkeleton->isChecked())
        {
            SoNode* s = CGALCoinVisualization::CreateSkeletonVisualization(skeleton, mesh->getMesh(), false);
            skeletonSep->addChild(s);

        } else if (UI.radioButtonSegmentation->isChecked())
        {
            SoNode* s = CGALCoinVisualization::CreateSegmentationVisualization(skeleton, mesh->getMesh(), members, false);
            skeletonSep->addChild(s);
        }


        if (UI.checkBoxGraspingInterval->isChecked())
        {
            skeletonSep->addChild(SimoxCGAL::CGALCoinVisualization::CreateGraspIntervalVisualization(approach->getInterval(), mesh->getMesh()));
        }

        if (UI.checkBoxGraspingPlane->isChecked())
        {
            skeletonSep->addChild(CGALCoinVisualization::CreateProjectedPointsVisualization(approach->getInterval(), mesh->getMesh()));
        }

    }

    viewer->scheduleRedraw();
}

int SkeletonGraspPlannerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void SkeletonGraspPlannerWindow::quit()
{
    std::cout << "GraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}
/*
void SkeletonGraspPlannerWindow::loadObject()
{
    if (!objectFile.empty())
    {
        object = ObjectIO::loadManipulationObject(objectFile);
    }
}
*/
void SkeletonGraspPlannerWindow::loadData()
{
    resetSceneryAll();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("XML Files (*.xml)"));
    string file = std::string(fi.toAscii());
    if (file.empty())
    {
        return;
    }
    loadSegmentedObject(file);
}

void SkeletonGraspPlannerWindow::loadSegmentedObject(const std::string & filename)
{
    segmentedObjectFile = filename;

    try
    {
        MeshSkeletonDataPtr data = MeshSkeletonData::loadSkeletonData(segmentedObjectFile);
        if (data)
        {
            object = data->manipObject;
            mesh = data->surfaceMesh;
            skeleton = data->skeleton->getSkeleton();
            segmentation = data->segSkeleton->getSegmentedObject();
        }
    } catch (...)
    {
        VR_ERROR << "could not load file " << segmentedObjectFile << endl;
        return;
    }

    //verschiebe Endeffector
    //Eigen::Vector3f min = object->getCollisionModel()->getTriMeshModel()->boundingBox.getMin();
    //Eigen::Vector3f max = object->getCollisionModel()->getTriMeshModel()->boundingBox.getMax();


    initPlanner();

    UI.groupBoxSkeleton->setEnabled(true);
    UI.radioButtonNothing->setChecked(true);

    buildVisu();

    updateSkeletonInfo();
}

void SkeletonGraspPlannerWindow::initPlanner()
{
    if (!robot || !eef)
    {
        VR_ERROR << "no robot or eef" << endl;
        return;
    }
    bool verbose = UI.checkBoxVerbose->isChecked();

    qualityMeasure.reset(new GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    preshape = "";
    approach.reset(new ApproachMovementSkeleton(object, skeleton, mesh->getMesh(), segmentation, eef, preshape));
    approach->setVerbose(verbose);
    eefCloned = approach->getEEFRobotClone();

    if (robot && eef)
    {
        std::string name = "Grasp Planner - ";
        name += eef->getName();
        grasps.reset(new GraspSet(name, robot->getType(), eefName));
    }

    std::string preshapeNamePrecision = approach->getParameters().preshapeName[ApproachMovementSkeleton::PlanningParameters::Precision];
    std::string preshapeNamePower = approach->getParameters().preshapeName[ApproachMovementSkeleton::PlanningParameters::Power];

    if (!eef->hasPreshape(preshapeNamePower)) {
        VR_ERROR << "no power preshape (" << preshapeNamePower << ") defined in endeffector: " << eef->getName() << " of robot: " << robot->getName() << endl;
    }


    if (!eef->hasPreshape(preshapeNamePrecision)) {
        VR_ERROR << "no precision preshape (" << preshapeNamePrecision << ") defined in endeffector: " << eef->getName() << " of robot: " << robot->getName() << endl;
    }

    planner.reset(new SkeletonGraspPlanner(grasps, qualityMeasure, approach));
    planner->setVerbose(verbose);
}

void SkeletonGraspPlannerWindow::loadRobot()
{
    robot.reset();
    robot = RobotIO::loadRobot(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << endl;
        return;
    }

    eef = robot->getEndEffector(eefName);

    eefCloned = eef->createEefRobot(eef->getName(), eef->getName());

    eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);
    eefVisu->ref();
}

void SkeletonGraspPlannerWindow::planAll()
{
    float timeout = 0.0f;
    bool forceClosure = UI.checkBoxFoceClosure->isChecked();
    float quality = (float)UI.doubleSpinBoxQuality->value();
    int nrGrasps = 1e8; // all grasps

    planGrasps(timeout, forceClosure, quality, nrGrasps);
}

void SkeletonGraspPlannerWindow::plan()
{
    float timeout = UI.spinBoxTimeOut->value() * 1000.0f;
    bool forceClosure = UI.checkBoxFoceClosure->isChecked();
    float quality = (float)UI.doubleSpinBoxQuality->value();
    int nrGrasps = UI.spinBoxGraspNumber->value();
    planGrasps(timeout, forceClosure, quality, nrGrasps);
}

void SkeletonGraspPlannerWindow::planGrasps(float timeout, bool forceClosure, float quality, int nrGrasps)
{
    if (!mesh || !skeleton || !segmentation)
    {
        return;
    }

    if (!planner)
    {
        planner.reset(new SkeletonGraspPlanner(grasps, qualityMeasure, approach, quality, forceClosure));
        planner->setVerbose(UI.checkBoxVerbose->isChecked());
    } else
    {
        planner->setParams(quality, forceClosure);
    }

    int nr = planner->plan(nrGrasps, timeout);
    planner->getEvaluation().print();
    if (UI.checkBoxVerbose->isChecked())
        VR_INFO << " Grasp planned:" << nr << endl;
    int start = (int)grasps->getSize() - nrGrasps - 1;

    if (start < 0)
    {
        start = 0;
    }

    preshape = approach->getGraspPreshape();

    if (nr != 0) {
        // keine Griffe mehr möglich!
        for (int i = start; i < (int)grasps->getSize() - 1; i++)
        {
//            Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());
//            SoSeparator* sep1 = new SoSeparator();
//            SoMatrixTransform* mt = CoinVisualizationFactory::getMatrixTransformScaleMM2M(m);
//            sep1->addChild(mt);
//            sep1->addChild(eefVisu);
            graspsSep->addChild(CGALCoinVisualization::CreateGraspVisualization(grasps->getGrasp(i), object));
        }
    }

    if (grasps->getSize()<=0)
        UI.spinBoxGraspNumberPlanned->setEnabled(false);
    else
    {
        UI.spinBoxGraspNumberPlanned->setEnabled(true);
        UI.spinBoxGraspNumberPlanned->setRange(0,grasps->getSize()-1);
        // set to last valid grasp
        UI.spinBoxGraspNumberPlanned->setValue(grasps->getSize()-1);
        selectGrasp();
    }

    updateSkeletonInfo();
}



void SkeletonGraspPlannerWindow::closeEEF()
{
    contacts.clear();

    if (eefCloned && eefCloned->getEndEffector(eefName) && object)
    {
        contacts = eefCloned->getEndEffector(eefName)->closeActors(object);
        qualityMeasure->setContactPoints(contacts);
        float qual = qualityMeasure->getGraspQuality();
        bool isFC = qualityMeasure->isGraspForceClosure();
        std::stringstream ss;
        ss << std::setprecision(3);
        int currentGrasp = UI.spinBoxGraspNumberPlanned->value();
        ss << "Grasp Nr " << currentGrasp << "\nQuality: " << qual << "\nForce closure: ";

        if (isFC)
        {
            ss << "yes";
        }
        else
        {
            ss << "no";
        }

        UI.labelInfo->setText(QString(ss.str().c_str()));
    }

    buildVisu();
}

void SkeletonGraspPlannerWindow::openEEF()
{
    contacts.clear();

    if (eefCloned && eefCloned->getEndEffector(eefName))
    {   
        VirtualRobot::EndEffectorPtr endeff = eefCloned->getEndEffector(eefName);
        if (endeff->hasPreshape(preshape)) {
            endeff->setPreshape(preshape);

        } else {
            endeff->openActors();
        }
    }

    buildVisu();
}

void SkeletonGraspPlannerWindow::frictionConeVisu()
{
    buildVisu();
}

void SkeletonGraspPlannerWindow::colModel()
{
    buildVisu();
}

void SkeletonGraspPlannerWindow::showGrasps()
{
    buildVisu();
}

void SkeletonGraspPlannerWindow::save()
{
    if (!object)
    {
        return;
    }



    ManipulationObjectPtr objectM(new ManipulationObject(object->getName(), object->getVisualization()->clone(), object->getCollisionModel()->clone()));
//    ManipulationObjectPtr objectM = VirtualRobot::ManipulationObject::createFromMesh(triMeshRefined);
//    objectM->saveModelFiles("iv", fa);
//    objectM->setCollisionModel(object->getCollisionModel()->clone());
    objectM->addGraspSet(grasps);
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

void SkeletonGraspPlannerWindow::selectGrasp()
{
    if (!grasps || !object)
        return;

    int currentGrasp = UI.spinBoxGraspNumberPlanned->value();

    if (currentGrasp>=0 && currentGrasp<int(grasps->getSize()) && eefCloned && eefCloned->getEndEffector(eefName))
    {
        Eigen::Matrix4f mGrasp = grasps->getGrasp(currentGrasp)->getTcpPoseGlobal(object->getGlobalPose());
        eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getTcp(), mGrasp);
        eefCloned->getEndEffector(eefName)->setPreshape(preshape);
        //separator mit grasping changing
    }

    openEEF();
    closeEEF();
}

void SkeletonGraspPlannerWindow::setVerbose()
{
    bool v = UI.checkBoxVerbose->isChecked();
    if (approach)
        approach->setVerbose(v);
    if (planner)
        planner->setVerbose(v);
}
