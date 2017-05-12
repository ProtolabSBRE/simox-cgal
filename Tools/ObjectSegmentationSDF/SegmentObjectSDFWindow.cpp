
#include "SegmentObjectSDFWindow.h"
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <QFileDialog>
#include <QInputDialog>
#include <Eigen/Geometry>
#include "CGALMeshConverter.h"
#include "Visualization/CoinVisualization/CGALCoinVisualization.h"
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>

#include <Inventor/actions/SoLineHighlightRenderAction.h>
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

SegmentObjectSDFWindow::SegmentObjectSDFWindow(const std::string& objFile)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->objectFilename = objFile;

    //boost::filesystem::path p(objFile);
    //save_dir = p.parent_path().string();

    sceneSep = new SoSeparator;
    sceneSep->ref();
    segObjectSep = new SoSeparator;
    sceneSep->addChild(segObjectSep);
    sdfObjectSep = new SoSeparator;
    sceneSep->addChild(sdfObjectSep);
    objectSep = new SoSeparator;
    sceneSep->addChild(objectSep);

    setupUI();

    loadObject();
    buildVisu();
    viewer->viewAll();
}


SegmentObjectSDFWindow::~SegmentObjectSDFWindow()
{
    sceneSep->unref();
}

void SegmentObjectSDFWindow::setupUI()
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
    connect(UI.checkBoxSDF, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxSegments, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.radioButtonFullModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.radioButtonColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(reloadObject()));
    connect(UI.pushButtonBuild, SIGNAL(clicked()), this, SLOT(buildObject()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(saveSegmentedObject()));
    //connect(UI.comboBoxSegments, SIGNAL(currentIndexChanged(int)), this, SLOT(showSegmentedObject()));
    //connect(UI.radioButtonShowSegment, SIGNAL(clicked()), this, SLOT(showSegmentedObject()));
    //connect(UI.pushButtonChoseSegment, SIGNAL(clicked()), this, SLOT(choseSegments()));
    connect(UI.pushButtonScreenshot, SIGNAL(clicked()), this, SLOT(screenshot()));
}


void SegmentObjectSDFWindow::resetSceneryAll()
{
}


void SegmentObjectSDFWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SegmentObjectSDFWindow::buildVisu()
{
    SceneObject::VisualizationType colModel = (UI.radioButtonColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    objectSep->removeAllChildren();
    if (manipObject && UI.checkBoxManip->isChecked())
    {
        SoNode* n = CoinVisualizationFactory::getCoinVisualization(manipObject, colModel);
        if (n)
        {
            objectSep->addChild(n);
        }

    }

    segObjectSep->removeAllChildren();
    if (sdfMesh && polyMesh && UI.checkBoxSegments->isChecked())
    {
        SoNode *n = CGALCoinVisualization::CreateCoinVisualizationSegments(polyMesh, sdfMesh->getSegmentMap(), sdfMesh->getNrSegments());
        segObjectSep->addChild(n);
    }

    sdfObjectSep->removeAllChildren();
    if (sdfMesh && UI.checkBoxSDF->isChecked())
    {
        SoNode *n = CGALCoinVisualization::CreateCoinVisualizationSDF(polyMesh, sdfMesh->getSDFMap(), sdfMesh->getMaxSDF());
        sdfObjectSep->addChild(n);
    }

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

int SegmentObjectSDFWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void SegmentObjectSDFWindow::quit()
{
    std::cout << "SegmentObjectSDFWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void SegmentObjectSDFWindow::saveSegmentedObject()
{

}

void SegmentObjectSDFWindow::loadObject()
{
    manipObject = ObjectIO::loadManipulationObject(objectFilename);

    buildVisu();
    viewer->viewAll();
}


void SegmentObjectSDFWindow::colModel()
{
    buildVisu();
}


void SegmentObjectSDFWindow::reloadObject()
{
    //QString fi = QFileDialog::getOpenFileName(this, tr("Open Object"), QString(), tr("XML Files (*.xml)"));
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

    objectFilename = std::string(fi.toAscii());
    if (objectFilename.empty())
    {
        return;
    }

    loadObject();
}


void SegmentObjectSDFWindow::buildObject()
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

    VR_INFO << "Converting mesh to cgal structure..." << endl;

    polyMesh = CGALMeshConverter::ConvertToPolyhedronMesh(manipObject->getCollisionModel()->getTriMeshModel());

    VR_INFO << "Segmenting mesh..." << endl;

    sdfMesh.reset(new SimoxCGAL::MeshSDF(polyMesh));

    VR_INFO << "done." << endl;

    buildVisu();
}

void SegmentObjectSDFWindow::screenshot()
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

