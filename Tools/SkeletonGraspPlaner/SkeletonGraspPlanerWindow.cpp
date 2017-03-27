#include "SkeletonGraspPlanerWindow.h"
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



#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoMaterial.h>


//#include "SkeletonGraspPlanerViewerIO.h"
#include "GraspPlaner/Math.h"
#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include "Segmentation/Skeleton/MeshSkeletonData.h"

#include <sstream>


using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;
using namespace SimoxCGAL;

float TIMER_MS = 30.0f;

SkeletonGraspPlanerWindow::SkeletonGraspPlanerWindow(std::string& robFile, std::string& eefName, std::string& preshape, std::string& segmentedObjectFile)
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


SkeletonGraspPlanerWindow::~SkeletonGraspPlanerWindow()
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

void SkeletonGraspPlanerWindow::setupUI()
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
    connect(UI.checkBoxGCP, SIGNAL(clicked()), this, SLOT(buildVisu()));

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
    connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonLoadData, SIGNAL(clicked()), this, SLOT(loadData()));
//    connect(UI.radioButtonPreshapeAll, SIGNAL(clicked()), this, SLOT(setPreshape()));
//    connect(UI.radioButtonPreshapePrecision, SIGNAL(clicked()), this, SLOT(setPreshape()));
//    connect(UI.radioButtonPreshapePower, SIGNAL(clicked()), this, SLOT(setPreshape()));


    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
//    connect(UI.checkBoxGCP, SIGNAL(clicked()), this, SLOT(showGrasps()));
//    connect(UI.checkBoxPoints, SIGNAL(clicked()), this, SLOT(buildVisu()));

}


void SkeletonGraspPlanerWindow::resetSceneryAll()
{
    if (grasps)
    {
        grasps->removeAllGrasps();
    }

    graspsSep->removeAllChildren();

    //if (rns)
    //  rns->setJointValues(startConfig);
}


void SkeletonGraspPlanerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SkeletonGraspPlanerWindow::buildVisu()
{

    robotSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned && UI.checkBoxHand->isChecked())
    {
        if (eefCloned->getEndEffector(eefName)->hasPreshape(preshape))
        {
            RobotConfigPtr config = eefCloned->getEndEffector(eefName)->getPreshape(preshape);
            std::string gcp = "GCP";
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
//        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);
        visualizationObject = object->getVisualization<CoinVisualization>(colModel2);
        visualizationObject->colorize(VisualizationFactory::Color::Gray());
        visualizationObject->setTransparency(0.7f);
        SoNode* visualisationNode = visualizationObject->getCoinVisualization();

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }
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


        SoSeparator* g = this->approach->getVisualization();

        if (g)
        {
            skeletonSep->addChild(g);
        }

    }

    viewer->scheduleRedraw();
}

int SkeletonGraspPlanerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void SkeletonGraspPlanerWindow::quit()
{
    std::cout << "GraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}
/*
void SkeletonGraspPlanerWindow::loadObject()
{
    if (!objectFile.empty())
    {
        object = ObjectIO::loadManipulationObject(objectFile);
    }
}
*/
void SkeletonGraspPlanerWindow::loadData()
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

void SkeletonGraspPlanerWindow::loadSegmentedObject(const std::string & filename)
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
}

void SkeletonGraspPlanerWindow::initPlanner()
{
    qualityMeasure.reset(new GraspQualityMeasureWrenchSpace(object));
    qualityMeasure->calculateObjectProperties();

    preshape = "";
    approach.reset(new ApproachMovementSkeleton(object, skeleton, mesh->getMesh(), segmentation, eef, preshape));
    eefCloned = approach->getEEFRobotClone();

    if (robot && eef)
    {
        std::string name = "Grasp Planner - ";
        name += eef->getName();
        grasps.reset(new GraspSet(name, robot->getType(), eefName));
    }

    planner.reset(new SkeletonGraspPlanner(grasps, qualityMeasure, approach));
    planner->setVerbose(true);
}

void SkeletonGraspPlanerWindow::loadRobot()
{
    robot.reset();
    robot = RobotIO::loadRobot(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << endl;
        return;
    }

    eef = robot->getEndEffector(eefName);

    if (!eef->hasPreshape(POWER_GRASP)) {
        VR_ERROR << "no power preshape defined in endeffector: " << eef->getName() << " of robot: " << robot->getName() << endl;
    }


    if (!eef->hasPreshape(PRECISION_GRASP)) {
        VR_ERROR << "no precision preshape defined in endeffector: " << eef->getName() << " of robot: " << robot->getName() << endl;
    }

    eefCloned = eef->createEefRobot(eef->getName(), eef->getName());

    eefVisu = CoinVisualizationFactory::CreateEndEffectorVisualization(eef);
    eefVisu->ref();
}

void SkeletonGraspPlanerWindow::plan()
{
    if (!mesh || !skeleton || !segmentation)
    {
        return;
    }

    float timeout = UI.spinBoxTimeOut->value() * 1000.0f;
    bool forceClosure = UI.checkBoxFoceClosure->isChecked();
    float quality = (float)UI.doubleSpinBoxQuality->value();
    int nrGrasps = UI.spinBoxGraspNumber->value();
    planner.reset(new SkeletonGraspPlanner(grasps, qualityMeasure, approach, quality, forceClosure));

    int nr = planner->plan(nrGrasps, timeout);
    VR_INFO << " Grasp planned:" << nr << endl;
    int start = (int)grasps->getSize() - nrGrasps - 1;

    if (start < 0)
    {
        start = 0;
    }

    preshape = approach->getGraspPreshape();
//    grasps->setPreshape(preshape);

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

    // set to last valid grasp
    if (grasps->getSize() > 0 && eefCloned && eefCloned->getEndEffector(eefName))
    {
        Eigen::Matrix4f mGrasp = grasps->getGrasp(grasps->getSize() - 1)->getTcpPoseGlobal(object->getGlobalPose());
        eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getTcp(), mGrasp);
        eefCloned->getEndEffector(eefName)->setPreshape(preshape);
    }


    if (nrGrasps > 0)
    {
        openEEF();
        closeEEF();
    }
}



void SkeletonGraspPlanerWindow::closeEEF()
{
    contacts.clear();

    if (eefCloned && eefCloned->getEndEffector(eefName) && object)
    {
        contacts = eefCloned->getEndEffector(eefName)->closeActors(object);
        float qual = qualityMeasure->getGraspQuality();
        bool isFC = qualityMeasure->isGraspForceClosure();
        std::stringstream ss;
        ss << std::setprecision(3);
        ss << "Grasp Nr " << grasps->getSize() << "\nQuality: " << qual << "\nForce closure: ";

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

void SkeletonGraspPlanerWindow::openEEF()
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

void SkeletonGraspPlanerWindow::frictionConeVisu()
{
    buildVisu();
}

void SkeletonGraspPlanerWindow::colModel()
{
    buildVisu();
}

void SkeletonGraspPlanerWindow::showGrasps()
{
    buildVisu();
}

//void SkeletonGraspPlanerWindow::load()
//{
//    VR_INFO << "Loading object.." << std::endl;

//    //select Object!
//    QString path("/common/homes/students/koch/Dokumente/ba_eduard_koch");
//    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object File"), path, tr("XML Files (*.xml)"));
//    this->objectFile = std::string(fi.toStdString());
//    //delete old data
//    skeletonDefault->clear();
//    mesh.clear();

//    loadObject();
////    resetScenery();
////    initMesh();
//    VR_INFO << "Loading object done." << std::endl;

//    //load Models
//    VR_INFO << "Loading skeleton ...\n";

//    QString path_data("/common/homes/students/koch/Dokumente/ba_eduard_koch/skeletons_data");
//    QString fi_data = QFileDialog::getOpenFileName(this, tr("Open Skeleton File"), path_data, tr("XML Files (*.xml)"));
//    string file(fi_data.toLatin1());

//    cout << "file: " << file << endl;
//    double remesh = -1.;



//    //begin loading.
//    //load file
//    std::ifstream in(file.c_str());
//    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << file);

//    std::stringstream buffer;
//    buffer << in.rdbuf();
//    std::string objectXML(buffer.str());
//    in.close();


//    char* y = new char[objectXML.size() + 1];
//    strncpy(y, objectXML.c_str(), objectXML.size() + 1);


//    try {

//        rapidxml::xml_document<char> doc;    // character type defaults to char
//        doc.parse<0>(y);    // 0 means default parse flags
//        rapidxml::xml_node<char>* objectXMLFile = doc.first_node("Skeleton");
//        THROW_VR_EXCEPTION_IF(!objectXMLFile, "No <Skeleton> tag in XML definition");
//        //inside the <Skeleton/> node

//        string object(BaseIO::processNameAttribute(objectXMLFile));
//        cout << "\tobject: " << object << endl;

//        THROW_VR_EXCEPTION_IF(object.empty(), "no attribute 'name'\n");
//        THROW_VR_EXCEPTION_IF(object != this->object->getName(), "Wrong file is loaded.\n");

//        rapidxml::xml_node<>* remeshingNode = objectXMLFile->first_node("Remeshing", 0, false);
//        remesh = BaseIO::processFloatAttribute("value", remeshingNode);

//        rapidxml::xml_node<>* nodeCGALMesh = objectXMLFile->first_node("CGAL_Mesh", 0, false);

//        if (nodeCGALMesh)
//        {
//            rapidxml::xml_node<>* m = nodeCGALMesh->first_node("File", 0, false);
//            rapidxml::xml_attribute<>* m_att = m->first_attribute("file", 0, false);
//            string file(m_att->value());
//            createMeshCGAL(file);

//        } else
//        {
//            THROW_VR_EXCEPTION("No CGAL_Triangle_Mesh found.\n");
//        }


//        rapidxml::xml_node<>* nodeSkeletonDefault = objectXMLFile->first_node("SkeletonDefault", 0, false);

//        if (nodeSkeletonDefault)
//        {
//            rapidxml::xml_node<>* node_default = nodeSkeletonDefault->first_node("File", 0, false);
//            rapidxml::xml_attribute<>* attribute_file = node_default->first_attribute("file", 0, false);
//            string file(attribute_file->value());
//            skeletonDefault->setSkeleton(SkeletonIO::loadSkeleton(file));

//        } else
//        {
//            THROW_VR_EXCEPTION("No skeleton with default parameters found.\n");
//        }

////        rapidxml::xml_node<>* nodeSkeletonParams = objectXMLFile->first_node("SkeletonParameters", 0, false);

////        if (nodeSkeletonParams)
////        {
////            rapidxml::xml_node<>* node_default = nodeSkeletonParams->first_node("File", 0, false);
////            rapidxml::xml_attribute<>* attribute_file = node_default->first_attribute("file", 0, false);
////            string file(attribute_file->value());
////            polyhedronParameters.clear();
////            polyhedronParameters.setSkeleton(SkeletonIO::loadSkeleton(file));

////        }else
////        {
////            THROW_VR_EXCEPTION("No skeleton with own parameters found.\n");
////        }

//        rapidxml::xml_node<>* nodeSeg = objectXMLFile->first_node("Segmentation", 0, false);

//        if (nodeSeg)
//        {
//            rapidxml::xml_node<>* node = nodeSeg->first_node("File", 0, false);
//            rapidxml::xml_attribute<>* attribute_file = node->first_attribute("file", 0, false);
//            string file(attribute_file->value());
//            segmentation = SegmentedIO::loadSegmentation(file);

//        }else
//        {
//            THROW_VR_EXCEPTION("No segmentation found.\n");
//        }

//    }
//    catch (rapidxml::parse_error& e)
//    {
//        delete[] y;
//        THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
//                           << "Error message:" << e.what() << endl
//                           << "Position: " << endl << e.where<char>() << endl);
//    }
//    catch (VirtualRobotException& e)
//    {
//        cout << " ERROR while loading object" << endl;
//        cout << e.what();
//        return;
//    }

//    initPlanner();
//    loadTriMesh();

//    std::cout << "\tremesh: " <<  remesh << std::endl;

//    UI.groupBoxSkeleton->setEnabled(true);
//    UI.radioButtonNothing->setChecked(true);

//    VR_INFO << "Loading complete.\n";

//    buildVisu();
//}

void SkeletonGraspPlanerWindow::save()
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

void SkeletonGraspPlanerWindow::setPreshape()
{
    initPlanner();
    buildVisu();

}
