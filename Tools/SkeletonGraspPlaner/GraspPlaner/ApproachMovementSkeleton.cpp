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

#include "Math.h"

#define PRINT 1

using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;


ApproachMovementSkeleton::ApproachMovementSkeleton(VirtualRobot::SceneObjectPtr object, SkeletonPtr skeleton, SurfaceMeshPtr mesh, SegmentedObjectPtr segmentation, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape)
    : ApproachMovementGenerator(object, eef, graspPreshape), skeleton(skeleton), mesh(mesh), segmentation(segmentation), decider(new DeciderGraspPreshape)
{
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
#if PRINT
    cout << "------START-----" << endl;
    cout << "Segment size: " << segmentation->getObjectParts().size() << std::endl;
#endif

    bool index = false;
    currentSubpart = 0;


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


#if PRINT
    cout << "current segment: " << currentSubpart << endl;
    cout << "current vertex: " << currentSkeletonVertex << endl;
    cout << "------DONE-----" << endl;
#endif
   return true;
}

Eigen::Matrix4f ApproachMovementSkeleton::createNewApproachPose()
{
    Eigen::Matrix4f pose = getEEFPose();
    openHand();
    Eigen::Vector3f position = plane.p;
    Eigen::Vector3f approachDir = approachDirs.back();;
    approachDirs.pop_back();

    this->aporachDirGlobal = approachDir;

    // set new pose
    setEEFToApproachPose(position,approachDir);

    // move away until valid
    moveEEFAway(approachDir, 0.5f);

    Eigen::Matrix4f poseB = getEEFPose();

    setEEFPose(pose);
    return poseB;

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

    //SkeletonVertex vertex = subpart->sortedSkeletonPartIndex.at(currentSkeletonVertex);
//    bool endpoint = !subpart->skeletonPart.at(vertex)->endpoint;
    bool endpoint = false;

    if (!endpoint)
    {
        //kein Endpunkt
        bool valid = Math::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, approachMovementParameters.interval[PlanningParameters::Precision], pca, plane);
        bool preshape = decider->decidePrecisionPreshape(pca.t2);

        if (valid && preshape)
        {
            //precision
            graspPreshape = approachMovementParameters.preshapeName[PlanningParameters::Precision];
            calculateApproachesConnectionPoint();
            approachDirectionsCalculated = true;
            return true;
        }

        valid = Math::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, approachMovementParameters.interval[PlanningParameters::Power], pca, plane);
        preshape = decider->decidePrecisionPreshape(pca.t2);

        if (valid && preshape)
        {
            //power
            graspPreshape = approachMovementParameters.preshapeName[PlanningParameters::Power];
            calculateApproachesConnectionPoint();
            approachDirectionsCalculated = true;
            return true;

        }


    } else {

        //working on endpoints


    }


    approachDirectionsCalculated = true;
    return false;
}

bool ApproachMovementSkeleton::setEEFToApproachPose(const Eigen::Vector3f &position, const Eigen::Vector3f &approachDir)
{
    // target pose
    Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

    // position
    poseFinal.block(0, 3, 3, 1) = position;

    //target orientation
    Eigen::Vector3f z = approachDir;

    z.normalize();
    z *= -1.0f;

    Eigen::Vector3f y;

    if (approachDirs.size() % 2 == 0)
    {
        y = plane.n;

    } else {
        y = plane.n * (-1);
    }

    Eigen::Vector3f x;
    x = y.cross(z);
    x.normalize();


    poseFinal.block(0, 0, 3, 1) = x;
    poseFinal.block(0, 1, 3, 1) = y;
    poseFinal.block(0, 2, 3, 1) = z;

    setEEFPose(poseFinal);

    return true;

}

bool ApproachMovementSkeleton::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
{
    cout << "Moving away..." << endl;
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


    cout << "Moving away done. Steps needed: " << loop << endl;
    return true;
}

void ApproachMovementSkeleton::calculateApproachDirRound(PrincipalAxis3D &pca, bool endpoint)
{

    if (endpoint)
    {
        return;
    }

    Eigen::Vector3f a1 = pca.pca1;
    Eigen::Vector3f a2 = pca.pca1 * (-1);
    Eigen::Vector3f b1 = pca.pca2;
    Eigen::Vector3f b2 = pca.pca2 * (-1);
    approachDirs.push_back(a1);
    approachDirs.push_back(a1);
    approachDirs.push_back(a2);
    approachDirs.push_back(a2);
    approachDirs.push_back(b1);
    approachDirs.push_back(b1);
    approachDirs.push_back(b2);
    approachDirs.push_back(b2);

    Eigen::Vector3f a = Math::createMidVector(pca.pca1, pca.pca2);
    Eigen::Vector3f b = Math::createMidVector(pca.pca1  * (-1), pca.pca2);
    Eigen::Vector3f c = Math::createMidVector(pca.pca1  * (-1), pca.pca2 * (-1));
    Eigen::Vector3f d = Math::createMidVector(pca.pca1, pca.pca2 * (-1));

    approachDirs.push_back(a);
    approachDirs.push_back(a);
    approachDirs.push_back(b);
    approachDirs.push_back(b);
    approachDirs.push_back(c);
    approachDirs.push_back(c);
    approachDirs.push_back(d);
    approachDirs.push_back(d);
}

void ApproachMovementSkeleton::calculateApproachDirRectangular(PrincipalAxis3D &pca, bool endpoint)
{
    if (endpoint)
    {
        return;
    }

    approachDirs.push_back(pca.pca1);
    approachDirs.push_back(pca.pca1);

    Eigen::Vector3f approach = pca.pca1 * (-1);
    approachDirs.push_back(approach);
    approachDirs.push_back(approach);
}

void ApproachMovementSkeleton::calculateApproachesConnectionPoint()
{
    float ratio = pca.eigenvalue1 / pca.eigenvalue2;

    if (ratio < approachMovementParameters.roundThreshold)
    {
        calculateApproachDirRound(pca);

    } else {
        calculateApproachDirRectangular(pca);
    }
}

void ApproachMovementSkeleton::calculateApproachesEndpoint()
{
//    float ratio = pca.eigenvalue1 / pca.eigenvalue2;

//    if (ratio < RATIO_THREASHOLD)
//    {
//        calculateApproachDirRound(pca);

//    } else {
//        calculateApproachDirRectangular(pca);
//    }

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
            cout << "Not valid on subpart " << currentSubpart << endl;
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
        VR_INFO << "SegNr: " << currentSubpart << ", next Index: " << currentSkeletonVertex << endl;
        return true;
    }

    // no valid vertex on currentr subpart -> next segment
    currentSubpart++;
    currentSkeletonVertex = 0;
    VR_INFO << "## Next segment: " << currentSubpart << ", index 0" << endl;

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

SoSeparator* ApproachMovementSkeleton::getVisualization() {

//    visu->removeAllChildren();
    visu->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(plane.p, 5.f, 0.f, 1.f, 0.f, 0.f));
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
