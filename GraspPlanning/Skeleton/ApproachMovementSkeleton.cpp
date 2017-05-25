#include "ApproachMovementSkeleton.h"

#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include  "VirtualRobot/VirtualRobotImportExport.h"
#include "VirtualRobot/RobotConfig.h"
#include "VirtualRobot/MathTools.h"
#include "Eigen/SVD"

#include "Inventor/nodes/SoMatrixTransform.h"
#include "Inventor/nodes/SoTransform.h"
#include "Inventor/nodes/SoUnits.h"

#include "SkeletonVertexAnalyzer.h"

//#define PRINT 1

using namespace std;
using namespace VirtualRobot;

namespace SimoxCGAL
{

ApproachMovementSkeleton::ApproachMovementSkeleton(VirtualRobot::SceneObjectPtr object, SkeletonPtr skeleton, SurfaceMeshPtr mesh, SegmentedObjectPtr segmentation, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape)
    : ApproachMovementGenerator(object, eef, graspPreshape), skeleton(skeleton), mesh(mesh), segmentation(segmentation), decider(new DeciderGraspPreshape)
{
    verbose = true;
    name = "ApproachMovementSkeleton";
    visu = new SoSeparator;
    visu->ref();
    approachDirs = vector<Eigen::Vector3f>();
    init();   

//    currentSubpart = 0;
//    currentSkeletonVertex = 100;
//    calculatedApproaches = false;
}

ApproachMovementSkeleton::~ApproachMovementSkeleton()
{
    visu->unref();
}

bool ApproachMovementSkeleton::init()
{
    if (verbose)
    {
        VR_INFO << "------START-----" << endl;
        VR_INFO << "Segment size: " << segmentation->getObjectParts().size() << std::endl;
    }

    decider->setThicknessPrecision(approachMovementParameters.minThickness[PlanningParameters::Precision], approachMovementParameters.maxThickness[PlanningParameters::Precision]);
    decider->setThicknessPower(approachMovementParameters.minThickness[PlanningParameters::Power], approachMovementParameters.maxThickness[PlanningParameters::Power]);

    bool index = false;
    currentSubpart = 0;
    currentSkeletonVertex = 0;


    while (!index)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

        if (size_t(currentSubpart) >= segmentation->getObjectParts().size())
        {
            break;
        }

        if (subpart->palpable)
        {
            index = true;
            currentSkeletonVertex = 0;

        } else
        {
            currentSubpart++;

        }
    }

    approachDirectionsCalculated = false;


    if (verbose)
    {
        VR_INFO << "current segment: " << currentSubpart << endl;
        VR_INFO << "current vertex: " << currentSkeletonVertex << endl;
        VR_INFO << "------DONE-----" << endl;
    }
    return true;
}

bool ApproachMovementSkeleton::createNewApproachPose(Eigen::Matrix4f &poseResult)
{
    if (!currentVertexResult.valid)
    {
        VR_ERROR << "Current vertex result is not valid?!" << endl;
        return false;
    }
    if (verbose)
    {
        VR_INFO << "Create new approach pose" << endl;
    }

    Eigen::Matrix4f pose = getEEFPose();
    openHand();
    Eigen::Vector3f position = currentVertexResult.graspingPlane.p;
    Eigen::Vector3f approachDir = approachDirs.back();
    approachDirs.pop_back();


    if (verbose)
        VR_INFO << "Approaches left: " << approachDirs.size() << endl;

    Eigen::Vector3f dirY = currentVertexResult.graspingPlane.n;

    if (currentVertexResult.endpoint)
    {
        Eigen::Vector3f tmp = dirY;
        dirY = approachDir;
        approachDir = tmp * -1;// since in setEEFToApproachPose, the approachDir is multiplied with -1, we need to adapt our input
    }

    if (approachDirs.size() % 2 != 0)
    {
        dirY *= -1;
    }

    if (verbose)
    {
        VR_INFO << "Approach pose, position:" << position.transpose() << ", approach dir: " << approachDir.transpose() << endl;
    }

    setEEFToApproachPose(position, approachDir, dirY);

    // move away until valid
    bool ok = moveEEFAway(approachDir, 1.0f, 150);

    // move away from object (power grasps)
    if (ok && approachMovementParameters.retreatDistance[getCurrentGraspType()]>0)
    {
        if (verbose)
        {
            VR_INFO << "Retreating " << approachMovementParameters.retreatDistance[getCurrentGraspType()] << "mm" << endl;
        }
        VirtualRobot::SceneObjectSetPtr sos = eef_cloned->createSceneObjectSet();

        Eigen::Vector3f delta = approachDir * approachMovementParameters.retreatDistance[getCurrentGraspType()];
        Eigen::Matrix4f ep = getEEFPose();
        updateEEFPose(delta);
        if (eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
        {
            if (verbose)
            {
                VR_INFO << "Retreat pose in collision, restoring original pose" << endl;
            }
            setEEFPose(ep);
        }
    }


    poseResult = getEEFPose();
    setEEFPose(pose);

    return ok;
}

Eigen::Matrix4f ApproachMovementSkeleton::createNewApproachPose()
{
    Eigen::Matrix4f r;
    createNewApproachPose(r);
    return r;
}


int ApproachMovementSkeleton::nextVertexOnCurrentSubpart(float dist)
{
    int cur = currentSkeletonVertex;
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
    int nrVertices = (int)subpart->sortedSkeletonPartIndex.size();

    if (cur >= nrVertices-1)
        return -1;

    // get next vertex
    if (dist == 0.f)
    {
        return (cur+1);
    }

    SkeletonVertex index = subpart->sortedSkeletonPartIndex.at(cur);
    SkeletonVertex index_next;
    float tmp = dist;

    while (true)
    {
        int t = cur + 1;

        if (t < nrVertices)
        {
            index_next = subpart->sortedSkeletonPartIndex.at(t);
            double d = std::sqrt(CGAL::squared_distance((*skeleton)[index].point, (*skeleton)[index_next].point));

            tmp -= (float)d;

            if (tmp >= 0)
            {
                index = index_next;
                cur++;

            } else {
                return t;
            }


        } else {
            // t == nrVertices -> cur last valid index
            return cur;
        }

    }

    return -1;
}

bool ApproachMovementSkeleton::calculateApproachDirection()
{
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
    currentVertexResult.valid = false;

    SkeletonVertexResult resultPre = SkeletonVertexAnalyzer::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, approachMovementParameters.interval[PlanningParameters::Precision],verbose);
    SkeletonVertexResult resultPower = SkeletonVertexAnalyzer::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, approachMovementParameters.interval[PlanningParameters::Power],verbose);
    bool valid = resultPre.valid;
    bool preshapePrecisionOK = false;
    bool preshapePowerOK = false;

    if (valid)
    {
        preshapePrecisionOK = decider->decidePrecisionPreshape(resultPre.pca.t1, resultPre.pca.t2);
        preshapePowerOK = decider->decidePowerPreshape(resultPower.pca.t1, resultPower.pca.t2);
    }

    if (valid && preshapePrecisionOK && !preshapePowerOK)
    {
        if (verbose)
        {
            VR_INFO << "Next point: Precision grasp, pca-lambda1 = " << resultPre.pca.t1 << ", pca-lamba2 = " << resultPre.pca.t2 << endl;
            if (resultPre.endpoint)
                VR_INFO << "Next point: Endpoint" << endl;
            else
                VR_INFO << "Next point: Connection point" << endl;
        }
        //precision
        graspPreshape = approachMovementParameters.preshapeName[PlanningParameters::Precision];
        if (resultPre.endpoint)
            calculateApproachesEndpoint(resultPre.pca);
        else
            calculateApproachesConnectionPoint(resultPre.pca);
        approachDirectionsCalculated = true;
        currentVertexResult = resultPre;
        return true;
    }

//    SkeletonVertexResult resultPower = SkeletonVertexAnalyzer::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, approachMovementParameters.interval[PlanningParameters::Power],verbose);
    valid = resultPower.valid;

//    if (valid)
//        preshapeOK = decider->decidePowerPreshape(resultPower.pca.t1, resultPower.pca.t2);

    if (valid && preshapePowerOK)
    {
        if (verbose)
        {
            VR_INFO << "Next point: Power grasp, pca-lambda1 = " << resultPre.pca.t1 << ", pca-lamba2 = " << resultPre.pca.t2 << endl;
            if (resultPower.endpoint)
                VR_INFO << "Next point: Endpoint" << endl;
            else
                VR_INFO << "Next point: Connection point" << endl;
        }
        //power
        graspPreshape = approachMovementParameters.preshapeName[PlanningParameters::Power];
        if (resultPower.endpoint)
            calculateApproachesEndpoint(resultPower.pca);
        else
            calculateApproachesConnectionPoint(resultPower.pca);
        approachDirectionsCalculated = true;
        currentVertexResult = resultPower;
        return true;

    }

    if (verbose)
    {
        VR_INFO << "Could not determine valid grasp for skeleton point" << endl;
    }

    approachDirectionsCalculated = true;
    return false;
}

bool ApproachMovementSkeleton::setEEFToApproachPose(const Eigen::Vector3f &position, const Eigen::Vector3f &approachDir, const Eigen::Vector3f &dirY)
{
    // target pose
    Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

    // position
    poseFinal.block(0, 3, 3, 1) = position;

    //target orientation
    Eigen::Vector3f z = approachDir;

    z.normalize();
    z *= -1.0f;

    Eigen::Vector3f y = dirY;


    Eigen::Vector3f x;
    x = y.cross(z);
    x.normalize();



//    SoSeparator* sep = new SoSeparator;
//    sep->ref();

//    SoTransform* trans = new SoTransform;
//    trans->translation.setValue(position[0], position[1], position[2]);
//    sep->addChild(trans);

//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(z, 20.f, 2.f, VirtualRobot::CoinVisualizationFactory::Color::Blue()));
//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(y, 20.f, 2.f, VirtualRobot::CoinVisualizationFactory::Color::Green()));
//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(x, 20.f, 2.f, VirtualRobot::CoinVisualizationFactory::Color::Red()));


    poseFinal.block(0, 0, 3, 1) = x;
    poseFinal.block(0, 1, 3, 1) = y;
    poseFinal.block(0, 2, 3, 1) = z;

    setEEFPose(poseFinal);

    return true;

}

bool ApproachMovementSkeleton::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
{
    VirtualRobot::SceneObjectSetPtr sos = eef_cloned->createSceneObjectSet();
    if (!sos)
    {
        return false;
    }

    int loop = 0;
    Eigen::Vector3f delta = approachDir * step;

    while (loop < maxLoops && eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
    {
        updateEEFPose(delta);
        loop++;
    }

    //Abstand prüfern!
    if (loop>=maxLoops)
    {
        if (verbose)
        {
            VR_INFO << "Could not determine collision free pose for eef..." << endl;
        }
        return false;
    }


    if (verbose)
    {
        VR_INFO << "Moving away done. Steps needed: " << loop << endl;
    }
    return true;
}

void ApproachMovementSkeleton::calculateApproachDirRound(const PrincipalAxis3D &pca, bool endpoint)
{

    if (verbose)
    {
        VR_INFO << "Round approach dirs:\n";
        VR_INFO << "Current approches: " << approachDirs.size() << endl;
    }


    Eigen::Vector3f a1 = pca.pca1;
    Eigen::Vector3f b1 = pca.pca2;
    Eigen::Vector3f a2 = pca.pca1 * (-1);
    Eigen::Vector3f b2 = pca.pca2 * (-1);

    Eigen::Vector3f a = SkeletonVertexAnalyzer::createMidVector(pca.pca1, pca.pca2);
    Eigen::Vector3f b = SkeletonVertexAnalyzer::createMidVector(pca.pca1  * (-1), pca.pca2);
    Eigen::Vector3f c = SkeletonVertexAnalyzer::createMidVector(pca.pca1  * (-1), pca.pca2 * (-1));
    Eigen::Vector3f d = SkeletonVertexAnalyzer::createMidVector(pca.pca1, pca.pca2 * (-1));

    approachDirs.push_back(a1);
    approachDirs.push_back(b1);
    approachDirs.push_back(a2);
    approachDirs.push_back(b2);

    if (!endpoint)
    {
        approachDirs.push_back(a1);
        approachDirs.push_back(a2);
        approachDirs.push_back(b1);
        approachDirs.push_back(b2);

    }

    if (verbose)
    {
        VR_INFO << "a1:" << a1.transpose() << endl;
        VR_INFO << "a2:" << a2.transpose() << endl;
        VR_INFO << "b1:" << b1.transpose() << endl;
        VR_INFO << "b2:" << b2.transpose() << endl;
        VR_INFO << "a:" << a.transpose() << endl;
        VR_INFO << "b:" << b.transpose() << endl;
        VR_INFO << "c:" << c.transpose() << endl;
        VR_INFO << "d:" << d.transpose() << endl;
    }

    approachDirs.push_back(a);
    approachDirs.push_back(b);
    approachDirs.push_back(c);
    approachDirs.push_back(d);

    if (!endpoint)
    {
        approachDirs.push_back(a);
        approachDirs.push_back(b);
        approachDirs.push_back(c);
        approachDirs.push_back(d);
    }

    if (verbose)
    {
        VR_INFO << "---Approches: " << approachDirs.size() << endl;
    }

}

void ApproachMovementSkeleton::calculateApproachDirRectangular(const PrincipalAxis3D &pca, bool endpoint)
{
    if (verbose)
    {
        VR_INFO << "Rect approach dirs:\n";
    }

    approachDirs.push_back(pca.pca1);
    approachDirs.push_back(pca.pca1);


    Eigen::Vector3f approach = pca.pca1 * (-1);


    if (!endpoint)
    {
        approachDirs.push_back(approach);
        approachDirs.push_back(approach);
    }

    if (verbose)
    {
        VR_INFO << "pca1:" << pca.pca1.transpose() << endl;
        VR_INFO << "-pca1:" << approach.transpose() << endl;
    }
}

void ApproachMovementSkeleton::calculateApproachesConnectionPoint(const PrincipalAxis3D &pca)
{
    float ratio = pca.eigenvalue1 / pca.eigenvalue2;

    if (verbose)
    {
        VR_INFO << "Approach dirs on Connection Point, ratio:" << ratio << endl;
    }

    if (ratio < approachMovementParameters.roundThreshold)
    {
        calculateApproachDirRound(pca, false);

    } else {
        calculateApproachDirRectangular(pca, false);
    }
}

void ApproachMovementSkeleton::calculateApproachesEndpoint(const PrincipalAxis3D &pca)
{
    float ratio = pca.eigenvalue1 / pca.eigenvalue2;

    if (verbose)
    {
        VR_INFO << "Approach dirs on EndPoint, ratio:" << ratio << endl;
    }

    if (ratio < approachMovementParameters.roundThreshold)
    {
        calculateApproachDirRound(pca, true);

    } else {
        calculateApproachDirRectangular(pca, true);
    }
}

bool ApproachMovementSkeleton::isValid()
{
    int sizeSubparts = segmentation->getObjectParts().size();

    if (currentSubpart < sizeSubparts)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

        if (size_t(currentSkeletonVertex) < subpart->sortedSkeletonPartIndex.size())
        {
            return true;
        }

    }

    return false;
}

bool ApproachMovementSkeleton::moreSegmentsAvailable()
{
    return (currentSubpart < int(segmentation->getObjectParts().size()));
}

ApproachMovementSkeleton::PlanningParameters ApproachMovementSkeleton::getParameters()
{
    return approachMovementParameters;
}

void ApproachMovementSkeleton::setParameters(ApproachMovementSkeleton::PlanningParameters &p)
{
    approachMovementParameters = p;
    if (decider)
    {
        decider->setThicknessPrecision(approachMovementParameters.minThickness[PlanningParameters::Precision], approachMovementParameters.maxThickness[PlanningParameters::Precision]);
        decider->setThicknessPower(approachMovementParameters.minThickness[PlanningParameters::Power], approachMovementParameters.maxThickness[PlanningParameters::Power]);
    }
}

bool ApproachMovementSkeleton::setNextIndex()
{
    //approaches schon generiert?
    if (!approachDirectionsCalculated && isValid())
    {
        return true;
    }

    while (!finished())
    {

        bool ok = nextSkeletonVertex();
        if (ok)
        {
            approachDirectionsCalculated = false;
            return true;
        }

        if (!ok || !isValid())
        {
            VR_INFO << "Not valid on subpart " << currentSubpart << endl;
            //return false;
        }
    }

    approachDirectionsCalculated = false;
    return false;
}

bool ApproachMovementSkeleton::finished()
{
    int nrObjectParts = int(segmentation->getObjectParts().size());

    if (currentSubpart >= nrObjectParts)
    {
        return true;
    }

    if (currentSubpart == nrObjectParts-1)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
        int nrVertices = (int)subpart->sortedSkeletonPartIndex.size();
        if (currentSkeletonVertex >= nrVertices)
            return true;
    }
    return false;
}

int ApproachMovementSkeleton::getCurrentSegment()
{
    return (int(currentSubpart));
}

int ApproachMovementSkeleton::getCurrentVertex()
{
    return (int(currentSkeletonVertex));
}

int ApproachMovementSkeleton::getSegmentCount()
{
    return int(segmentation->getObjectParts().size());
}

int ApproachMovementSkeleton::getVertexCount(int segNr)
{
    if (!segmentation || segNr < 0 || segNr >= getSegmentCount())
        return -1;
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
    if (!subpart)
        return -1;

    int nrVertices = (int)subpart->sortedSkeletonPartIndex.size();

    return nrVertices;
}


bool ApproachMovementSkeleton::nextSkeletonVertex()
{
    if (!segmentation || currentSubpart >= int(segmentation->getObjectParts().size()))
    {
        VR_ERROR << "Could not determine next vertex" << endl;
        return false;
    }

    //SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
    approachDirectionsCalculated = false;

    int newIndex = nextVertexOnCurrentSubpart(approachMovementParameters.skeletonSamplingLength);
    if (newIndex == currentSkeletonVertex)
    {
       VR_WARNING << "Current index == next index on subpart " << currentSubpart << ", indx = " << currentSkeletonVertex << endl;
    }
    currentSkeletonVertex = newIndex;

    if (currentSkeletonVertex>=0)
    {
        if (verbose)
        {
            VR_INFO << "SegNr: " << currentSubpart << ", next Index: " << currentSkeletonVertex << endl;
        }
        return true;
    }

    // no valid vertex on currentr subpart -> next segment
    currentSubpart++;
    currentSkeletonVertex = 0;
    if (verbose)
    {
        VR_INFO << "## Next segment: " << currentSubpart << ", index 0" << endl;
    }

    return isValid();
    /*if (!isValid())
    {
        nextSkeletonVertex();
    }*/
}

string ApproachMovementSkeleton::getGraspPreshape()
{
    return graspPreshape;
}

ApproachMovementSkeleton::PlanningParameters::GraspType ApproachMovementSkeleton::getCurrentGraspType()
{
    if (graspPreshape==approachMovementParameters.preshapeName[PlanningParameters::Precision])
        return ApproachMovementSkeleton::PlanningParameters::Precision;
    if (graspPreshape==approachMovementParameters.preshapeName[PlanningParameters::Power])
        return ApproachMovementSkeleton::PlanningParameters::Power;

    VR_ERROR << "nyi" << endl;
    return ApproachMovementSkeleton::PlanningParameters::Power;
}

SoSeparator* ApproachMovementSkeleton::getVisualization() {

//    visu->removeAllChildren();
    visu->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(currentVertexResult.graspingPlane.p, 5.f, 0.f, 1.f, 0.f, 0.f));
    return visu;
}

RobotNodePtr ApproachMovementSkeleton::getTCP()
{
    return eef_cloned->getPreshape(graspPreshape)->getTCP();
}

bool ApproachMovementSkeleton::setEEFPose(const Eigen::Matrix4f& pose)
{

    eefRobot->setGlobalPoseForRobotNode(eef_cloned->getPreshape(graspPreshape)->getTCP(), pose);
    return true;
}

bool ApproachMovementSkeleton::updateEEFPose(const Eigen::Vector3f& deltaPosition)
{
    Eigen::Matrix4f deltaPose;
    deltaPose.setIdentity();
    deltaPose.block(0, 3, 3, 1) = deltaPosition;
    return updateEEFPose(deltaPose);
}

void ApproachMovementSkeleton::setVerbose(bool v)
{
    verbose = v;
}

bool ApproachMovementSkeleton::updateEEFPose(const Eigen::Matrix4f& deltaPose)
{

    Eigen::Matrix4f pose = getEEFPose();
    pose = deltaPose * pose;
    return setEEFPose(pose);
}

Eigen::Matrix4f ApproachMovementSkeleton::getEEFPose()
{

     return eef_cloned->getPreshape(graspPreshape)->getTCP()->getGlobalPose();
}

int ApproachMovementSkeleton::getApproachesNumber()
{
    return approachDirs.size();
}

SkeletonVertexResult ApproachMovementSkeleton::getInterval()
{
    return currentVertexResult;
}

}

