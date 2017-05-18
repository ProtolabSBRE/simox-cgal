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
#include <GraspPlanning/GraspQuality/GraspEvaluationPoseUncertainty.h>

#include "GraspPlanning/Skeleton/SkeletonVertexAnalyzer.h"
#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include "Segmentation/Skeleton/MeshSkeletonData.h"
#include <QProgressDialog>
#include <sstream>

#include <CGALMeshConverter.h>
#include "ui_SkeletonGraspPlannerOptions.h"

using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

SkeletonGraspPlannerWindow::SkeletonGraspPlannerWindow(std::string& robFile, std::string& eefName, std::string& segmentedObjectFile)
    : QMainWindow(NULL), skeleton(new Skeleton), segmentation(new SegmentedObject())
{
    VR_INFO << " start " << endl;

    // init the random number generator
    srand(time(NULL));

    this->robotFile = robFile;
    this->segmentedObjectFile = segmentedObjectFile;
    this->eefName = eefName;
    this->currentPreshape = "";
    eefVisu = NULL;

    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    graspsSep = new SoSeparator;
    skeletonSep = new SoSeparator;
    graspsSep->ref();
    surfaceGrasp = new SoSeparator;
    surfaceGrasp->ref();

    test = new SoSeparator;
    test->ref();

    sceneSep->addChild(robotSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    sceneSep->addChild(test);
    sceneSep->addChild(surfaceGrasp);

    setupUI();

    loadRobot();
    loadSegmentedObject(segmentedObjectFile);

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

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();
    viewer->setAccumulationBuffer(true);
    viewer->setAntialiasing(true, 4);

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
    connect(UI.pushButtonPlanBatch, SIGNAL(clicked()), this, SLOT(planObjectBatch()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonLoadData, SIGNAL(clicked()), this, SLOT(loadData()));
    connect(UI.pushButtonOptions, SIGNAL(clicked()), this, SLOT(plannerOptions()));
    connect(UI.spinBoxGraspNumberPlanned, SIGNAL(valueChanged(int)), this, SLOT(selectGrasp()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
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

    openEEF();
    if (eefCloned)
    {
        eefCloned->setGlobalPose(Eigen::Matrix4f::Identity());
    }

    currentPreshape = "";

    UI.spinBoxGraspNumberPlanned->setValue(0);
    UI.spinBoxGraspNumberPlanned->setEnabled(false);

    initPlanner();

    updateSkeletonInfo();
}


void SkeletonGraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SkeletonGraspPlannerWindow::plannerOptions()
{
    if (!approach)
    {
        VR_ERROR << "No approach data" << endl;
        return;
    }
    // setup window
    Ui::SkeletonGraspPlannerOptions UICreate;
    QDialog diag;
    UICreate.setupUi(&diag);
    std::string tcp = "<none>";
    std::string eefS = "<none>";
    if (eef)
    {
        eefS = eef->getName();
        RobotNodePtr tcpNode = eef->getTcp();
        if (tcpNode)
            tcp = tcpNode->getName();
    }
    UICreate.labelEEF->setText(QString("End Effector: ") + QString(eefS.c_str()));
    UICreate.labelTCP->setText(QString("TCP: ") + QString(tcp.c_str()));

    ApproachMovementSkeleton::PlanningParameters p = approach->getParameters();
    UICreate.dsb_RoundRectRatio->setValue(p.roundThreshold);
    UICreate.dsb_SkeletonDist->setValue(p.skeletonSamplingLength);
    UICreate.dsbPr_MinThick->setValue(p.minThickness[ApproachMovementSkeleton::PlanningParameters::Precision]);
    UICreate.dsbPr_MaxThick->setValue(p.maxThickness[ApproachMovementSkeleton::PlanningParameters::Precision]);
    UICreate.dsbPr_IntervalWidth->setValue(p.interval[ApproachMovementSkeleton::PlanningParameters::Precision]);

    UICreate.dsbPo_MinThick->setValue(p.minThickness[ApproachMovementSkeleton::PlanningParameters::Power]);
    UICreate.dsbPo_MaxThick->setValue(p.maxThickness[ApproachMovementSkeleton::PlanningParameters::Power]);
    UICreate.dsbPo_IntervalWidth->setValue(p.interval[ApproachMovementSkeleton::PlanningParameters::Power]);
    UICreate.dsbPo_RetreatDistance->setValue(p.retreatDistance[ApproachMovementSkeleton::PlanningParameters::Power]);

    if (diag.exec())
    {
        p.roundThreshold = UICreate.dsb_RoundRectRatio->value();
        p.minThickness[ApproachMovementSkeleton::PlanningParameters::Precision] = UICreate.dsbPr_MinThick->value();
        p.maxThickness[ApproachMovementSkeleton::PlanningParameters::Precision] = UICreate.dsbPr_MaxThick->value();
        p.interval[ApproachMovementSkeleton::PlanningParameters::Precision] = UICreate.dsbPr_IntervalWidth->value();

        p.minThickness[ApproachMovementSkeleton::PlanningParameters::Power] = UICreate.dsbPo_MinThick->value();
        p.maxThickness[ApproachMovementSkeleton::PlanningParameters::Power] = UICreate.dsbPo_MaxThick->value();
        p.interval[ApproachMovementSkeleton::PlanningParameters::Power] = UICreate.dsbPo_IntervalWidth->value();
        p.retreatDistance[ApproachMovementSkeleton::PlanningParameters::Power] = UICreate.dsbPo_RetreatDistance->value();
        p.skeletonSamplingLength = UICreate.dsb_SkeletonDist->value();

        approach->setParameters(p);
    }
}


void SkeletonGraspPlannerWindow::buildVisu()
{

    robotSep->removeAllChildren();
    surfaceGrasp->removeAllChildren();
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
        if (eefCloned->getEndEffector(eefName)->hasPreshape(currentPreshape))
        {
            RobotConfigPtr config = eefCloned->getEndEffector(eefName)->getPreshape(currentPreshape);
            std::string gcp = "GCP";
            if (config->getTCP())
                config->getTCP()->showCoordinateSystem(UI.checkBoxGCP->isChecked(), 0.5f, &gcp);
        }

        visualizationRobot = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();

        // show projected surface point
        if (mesh && currentGrasp)
        {
            CGALPolyhedronMeshPtr poly = CGALMeshConverter::convertSurface2PolyhedronMesh(mesh);
            if (poly)
                surfaceGrasp->addChild(CGALCoinVisualization::CreateGraspOnSurfaceVisualization(currentGrasp, eefCloned->getEndEffector(eefName), object, poly));
        }

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
            color->transparency = UI.horizontalSliderTr->value()/100.0f;
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

void SkeletonGraspPlannerWindow::loadData()
{
    //QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("Segmented Object XML Files (*.soxml)"));
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    QStringList nameFilters;
    nameFilters << "Segmented Object XML Files (*.soxml)"
                << "XML Files (*.xml)"
                   "All Files (*.*)";
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

    string file = std::string(fi.toAscii());
    if (file.empty())
        return;

    loadSegmentedObject(file);
}

bool SkeletonGraspPlannerWindow::loadSegmentedObject(const std::string & filename)
{
    segmentedObjectFile = filename;
    VR_INFO << "Loading segmented object from " << filename << std::endl;
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
        return false;
    }

    initPlanner();

    UI.groupBoxSkeleton->setEnabled(true);
    UI.radioButtonNothing->setChecked(true);

    graspsSep->removeAllChildren();
    buildVisu();

    updateSkeletonInfo();
    return true;
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

    currentPreshape = "";
    bool oldValues = false;
    ApproachMovementSkeleton::PlanningParameters p;
    if (approach)
    {
        oldValues = true;
        p = approach->getParameters();
    }
    approach.reset(new ApproachMovementSkeleton(object, skeleton, mesh->getMesh(), segmentation, eef, currentPreshape));
    approach->setVerbose(verbose);
    if (oldValues)
        approach->setParameters(p);
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

    // just set to an initial value, finally updated in selectGrasp()
    currentPreshape = approach->getGraspPreshape();

    if (nr != 0) {
        // keine Griffe mehr möglich!
        VR_INFO << "Create visualization of all grasps..." << endl;
        CGALPolyhedronMeshPtr poly = CGALMeshConverter::convertSurface2PolyhedronMesh(mesh);
        if (poly)
            graspsSep->addChild(CGALCoinVisualization::CreateGraspsOnSurfaceVisualization(grasps,eefCloned->getEndEffector(eefName),object,poly,2.0f,3.0f,20.0f));
        VR_INFO << "Create visualization of all grasps... done" << endl;

        /*for (int i = start; i < (int)grasps->getSize() - 1; i++)
        {
            graspsSep->addChild(CGALCoinVisualization::CreateGraspVisualization(grasps->getGrasp(i), object));
        }*/
    }

    if (grasps->getSize()<=0)
        UI.spinBoxGraspNumberPlanned->setEnabled(false);
    else
    {
        UI.spinBoxGraspNumberPlanned->setEnabled(true);
        UI.spinBoxGraspNumberPlanned->setRange(0,grasps->getSize()-1);
        // set to last valid grasp
        UI.spinBoxGraspNumberPlanned->setValue(grasps->getSize()-1);
        //selectGrasp();
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
        ss << "\nPreshape:" << currentPreshape;

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
        if (endeff->hasPreshape(currentPreshape))
        {
            endeff->setPreshape(currentPreshape);
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
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    QStringList nameFilters;
    nameFilters << "Manipulation Files (*.moxml)"
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

    saveToFile(fi.toStdString());
}

void SkeletonGraspPlannerWindow::saveToFile(std::string filepath)
{
    if (!object)
    {
        return;
    }

    ManipulationObjectPtr objectM(new ManipulationObject(object->getName(), object->getVisualization()->clone(), object->getCollisionModel()->clone()));
    objectM->addGraspSet(grasps);
    std::string objectFile = filepath;
    bool ok = false;

    try
    {
        VR_INFO << "Saving to " << objectFile << std::endl;
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
        VirtualRobot::GraspPtr g = grasps->getGrasp(currentGrasp);
        this->currentGrasp = g;
        applyGrasp(g, eefCloned, eefCloned->getEndEffector(eefName));
        if (UI.checkBoxEvaluateGrasps->isChecked())
        {
            float a,b;
            evaluateGrasp(g, eefCloned, eefCloned->getEndEffector(eefName), 100, a, b);
            VR_INFO << "Robustness: avg quality:" << a << endl;
            VR_INFO << "Robustness: avg fc rate:" << b << endl;
        }
    } else
    {
        openEEF();
    }
}

bool SkeletonGraspPlannerWindow::evaluateGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef, int nrEvalLoops, float &storeAvgRate, float &storeAvgForceClosureRate)
{
    if (!g || !eefRobot || !eef)
        return false;

    storeAvgRate = 0;
    storeAvgForceClosureRate = 0;

    GraspEvaluationPoseUncertaintyPtr eval(new GraspEvaluationPoseUncertainty(GraspEvaluationPoseUncertainty::PoseUncertaintyConfig()));

    auto poseevalresult = eval->evaluateGrasp(g, eef, object, qualityMeasure, nrEvalLoops);
    storeAvgRate = poseevalresult.avgQuality;
    storeAvgForceClosureRate = poseevalresult.forceClosureRate;

    return true;
}

void SkeletonGraspPlannerWindow::applyGrasp(VirtualRobot::GraspPtr g, VirtualRobot::RobotPtr eefRobot, VirtualRobot::EndEffectorPtr eef)
{
    if (!g)
        return;
    currentPreshape = g->getPreshapeName();
    Eigen::Matrix4f mGrasp = g->getTcpPoseGlobal(object->getGlobalPose());
    //eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getTcp(), mGrasp);
    if (eefRobot && eef)
        eefRobot->setGlobalPoseForRobotNode(eef->getTcp(), mGrasp);

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

void SkeletonGraspPlannerWindow::planObjectBatch()
{
    QString fi = QFileDialog::getExistingDirectory(this, tr("Select Base Directory"), QString());
    qApp->processEvents();
    VR_INFO << "Searching for all .soxml files in " << fi.toStdString() << std::endl;
    if (fi.isEmpty())
    {
        return;
    }
    QStringList paths;
    for (boost::filesystem::recursive_directory_iterator end, dir(fi.toUtf8().data());
         dir != end ; ++dir)
    {
        std::string path(dir->path().c_str());

        // search for all statechart group xml files
        if (dir->path().extension() == ".soxml")
        {
            paths << dir->path().c_str();
        }

    }
    paths.removeDuplicates();
    VR_INFO << "Found:  " << paths.join(", ").toStdString() << std::endl;
    boost::filesystem::path resultsCSVPath("graspplanningresults-" + robot->getName() + ".csv");
    resultsCSVPath = boost::filesystem::absolute(resultsCSVPath);
    std::ofstream fs(resultsCSVPath.string().c_str(), std::ofstream::out);
    fs << "object," << planner->getEvaluation().GetCSVHeader() << "RobustnessAvgQuality,RobustnessAvgForceClosureRate" << std::endl;
    QProgressDialog progress("Calculating grasps...", "Abort", 0, paths.size(), this);
    progress.setWindowModality(Qt::WindowModal);
    progress.show();
    int i = 0;
    progress.setValue(0);
    qApp->processEvents();
    for(auto& path :  paths)
    {
        try
        {
            //resetSceneryAll();
            if(loadSegmentedObject(path.toStdString()))
            {
                planAll();
                saveToFile(boost::filesystem::path(path.toStdString()).replace_extension(".moxml").string());
                float avgRate = 0;
                float avgForceClosureRate = 0;
                for(VirtualRobot::GraspPtr& g : planner->getPlannedGrasps())
                {
                    float a,b;
                    if (!evaluateGrasp(g, eefCloned, eefCloned->getEndEffector(eefName), 100, a, b))
                        continue;
                    avgRate += a;
                    avgForceClosureRate += b;
                }
                fs << object->getName() << "," << planner->getEvaluation().toCSVString() << "," << (avgRate/planner->getPlannedGrasps().size()) << "," << (avgForceClosureRate/planner->getPlannedGrasps().size()) << std::endl;
            }
        }
        catch(std::exception & e)
        {
            VR_ERROR << "Failed to plan for " << path.toStdString() << "\nReason: \n" << e.what() << std::endl;
        }
        progress.setValue(++i);
        qApp->processEvents();
        if (progress.wasCanceled())
            break;
    }
    VR_INFO << "Saving CSV results to " << resultsCSVPath.string() << std::endl;
}
