
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

    //connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonLoadSegmented, SIGNAL(clicked()), this, SLOT(reloadSegmentedObject()));
    //connect(UI.comboBoxSegments, SIGNAL(currentIndexChanged(int)), this, SLOT(showSegmentedObject()));
    //connect(UI.radioButtonShowSegment, SIGNAL(clicked()), this, SLOT(showSegmentedObject()));
    //connect(UI.pushButtonChoseSegment, SIGNAL(clicked()), this, SLOT(choseSegments()));
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
/*
void SegmentedObjectViewerWindow::showSegmentedObject()
{
    buildVisu();
}
*/


void SegmentedObjectViewerWindow::colModel()
{
    buildVisu();
}


void SegmentedObjectViewerWindow::reloadSegmentedObject()
{
    //QString fi = QFileDialog::getOpenFileName(this, tr("Open Segmented Object"), QString(), tr("XML Files (*.xml)"));
    QString fi;
    QFileDialog dialog(this);
    dialog.setFileMode(QFileDialog::ExistingFile);
    dialog.setAcceptMode(QFileDialog::AcceptOpen);
    QStringList nameFilters;
    nameFilters << "XML Files (*.xml)"
                << "Manipulation Files (*.moxml)"
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

    segmentedFilename = std::string(fi.toAscii());
    if (segmentedFilename.empty())
        return;

    loadSegmentedObject();
}


