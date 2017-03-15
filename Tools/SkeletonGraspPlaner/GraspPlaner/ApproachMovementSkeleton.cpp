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
    : ApproachMovementGenerator(object, eef, graspPreshape), decider(new DeciderGraspPreshape), skeleton(skeleton), segmentation(segmentation), mesh(mesh)
{
    name = "ApproachMovementSkeleton";
    sep = new SoSeparator;
    sep->ref();
    validGrasp = true;
    approachDirs = vector<Eigen::Vector3f>();
    init();
//    currentSubpart = 0;
//    currentSkeletonVertex = 100;
//    calculatedApproaches = false;
}

ApproachMovementSkeleton::~ApproachMovementSkeleton()
{
    sep->unref();
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

        if (currentSubpart >= segmentation->getObjectParts().size())
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

    calculatedApproaches = false;


#if PRINT
        cout << "current segment: " << currentSubpart << endl;
        cout << "current vertex: " << currentSkeletonVertex << endl;
        cout << "------DONE-----" << endl;
#endif
}

Eigen::Matrix4f ApproachMovementSkeleton::createNewApproachPose()
{
    std::cout << "createNewApproachPose" << std::endl;
    Eigen::Matrix4f pose = getEEFPose();
    openHand();
    Eigen::Vector3f position;
    Eigen::Vector3f approachDir;

    position = plane.p;
    approachDir = approachDirs.back();
    approachDirs.pop_back();

    std::cout << "remaining: " << approachDirs.size() << std::endl;
    std::cout << "position plane.p: " << position << std::endl;

    this->aporachDirGlobal = approachDir;
//    eef_cloned->setPreshape();

    // set new pose
    setEEFToApproachPose(position,approachDir);

    // move away until valid
    moveEEFAway(approachDir, 0.5f);

    Eigen::Matrix4f poseB = getEEFPose();

    setEEFPose(pose);

    std::cout << "createApproach done." << std::endl;
    return poseB;

}

void ApproachMovementSkeleton::next()
{
    cout << "next" << endl;
    cout << "current1: " << currentSubpart << endl;
    cout << "current2: " << currentSkeletonVertex << endl;

    if (currentSubpart >= segmentation->getObjectParts().size())
    {
        return;
    }

    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));
    calculatedApproaches = false;

    if (currentSkeletonVertex < subpart->sortedSkeletonPartIndex.size())
    {
        cout << "hallo" << endl;
        currentSkeletonVertex = samplingSkeleton(SAMPLING_LENGTH);
        return;
    }

    cout << "hallo1" << endl;

    currentSubpart++;
    currentSkeletonVertex = 0;

    if (!isValid())
    {
        next();
    }
}

bool ApproachMovementSkeleton::setNextIndex()
{
    cout << "setNextIndex" << endl;
    //approaches schon generiert?
    if (!calculatedApproaches && isValid())
    {
        cout << "nicht berechnet false" << endl;
        return true;
    }

    next();

    if (!isValid())
    {
        cout << currentSubpart << endl;
        cout << "ENDE false" << endl;
        return false;
    }

    calculatedApproaches = false;
    cout << "setNextIndex true" << endl;
    return true;
}

int ApproachMovementSkeleton::samplingSkeleton(float dist)
{
    int cur = currentSkeletonVertex;
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

    if (dist == 0.f)
    {
        cur++;
        std::cout << "cur: " << cur << std::endl;
        return cur;
    }


    SkeletonVertex index = subpart->sortedSkeletonPartIndex.at(cur);
    SkeletonVertex index_next;
    float tmp = dist;

    while (true)
    {
        int t = cur + 1;

        if (t < subpart->sortedSkeletonPartIndex.size())
        {
            std::cout << "cur:" << cur << std::endl;
            std::cout << "t:" << t << std::endl;
            index_next = subpart->sortedSkeletonPartIndex.at(t);
            double d = std::sqrt(CGAL::squared_distance((*skeleton)[index].point, (*skeleton)[index_next].point));
            std::cout << "d: " << d << std::endl;

            tmp -= (float)d;

            if (tmp >= 0)
            {
                index = index_next;
                cur++;

            } else {

                return t;
            }


        } else {

            return t;
        }

    }



}

bool ApproachMovementSkeleton::isValid()
{
    int sizeSubparts = segmentation->getObjectParts().size();

    if (currentSubpart < sizeSubparts)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

        if (currentSkeletonVertex < subpart->sortedSkeletonPartIndex.size())
        {
            return true;
        }

    }

    return false;
}

bool ApproachMovementSkeleton::areMoreSegments()
{
    return (currentSubpart < segmentation->getObjectParts().size());
}

bool ApproachMovementSkeleton::calculateApproachDirection()
{
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

    std::cout << "palpalbe: " << subpart->palpable << std::endl;
    std::cout << "currentSubpart: " << currentSubpart << " of " << segmentation->getObjectParts().size() - 1 << std::endl;
    std::cout << "currentSkeletonVertexIndex: " << currentSkeletonVertex << std::endl;

    bool valid = Math::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, PRECISION_INTERVALL, pca, plane);
    bool preshape = decider->decidePrecisionPreshape(pca.t2);

    if (valid && preshape)
    {
        //precision
        graspPreshape = PRECISION_GRASP;
        calculateApproaches();
        calculatedApproaches = true;
        return true;
    }

    valid = Math::calculatePCA(skeleton, mesh, currentSkeletonVertex, subpart, POWER_INTERVALL, pca, plane);
    preshape = decider->decidePrecisionPreshape(pca.t2);

    if (valid && preshape)
    {
        //power
        graspPreshape = POWER_GRASP;
        calculateApproaches();
        calculatedApproaches = true;
        return true;

    }

    calculatedApproaches = true;
    return false;
}

bool ApproachMovementSkeleton::setEEFToApproachPose(const Eigen::Vector3f &position, const Eigen::Vector3f &approachDir)
{
    std::cout << "setEEFToApproachPose" << std::endl;

    // target pose
    Eigen::Matrix4f poseFinal = Eigen::Matrix4f::Identity();

    // position
    poseFinal.block(0, 3, 3, 1) = position;

    //target orientation
    Eigen::Vector3f z = approachDir;

    z.normalize();
    z *= -1.0f;

    Eigen::Vector3f y = plane.n;
    Eigen::Vector3f x;
    x = y.cross(z);
    x.normalize();


    poseFinal.block(0, 0, 3, 1) = x;
    poseFinal.block(0, 1, 3, 1) = y;
    poseFinal.block(0, 2, 3, 1) = z;

    setEEFPose(poseFinal);

    return true;

}

void ApproachMovementSkeleton::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
{
    VirtualRobot::SceneObjectSetPtr sos = eef_cloned->createSceneObjectSet();

    validGrasp = true;

    if (!sos)
    {
        return;
    }

    float length = 0.f;
    float radius = pca.t1;
    Eigen::Vector3f delta = approachDir * step;
    std::cout << "radi: " << radius << std::endl;


    if (!eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
    {
        std::cout << "valid" << std::endl;
        validGrasp = true;
        return;
    }

    Eigen::Matrix4f pose = getEEFPose();
    Eigen::Vector4f p0 = pose.col(3);
    Eigen::Vector3f ps(p0[0], p0[1], p0[2]);
//    std::cout << "pose" << std::endl << pose << std::endl;
//    std::cout << "pose: " << p0.transpose() << std::endl;

    while (eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
    {
//        std::cout << "length_while: " << length << std::endl;
        length += step;
        ApproachMovementGenerator::updateEEFPose(delta);

//        std::cout << "poseUpdate" << std::endl << getEEFPose() << std::endl;
        Eigen::Vector4f p = getEEFPose().col(3);
//        std::cout << "\nposeUP: " << p.transpose() << std::endl;
        Eigen::Vector3f tmp(p[0], p[1], p[2]);

        Eigen::Vector3f dif = tmp - ps;
//        std::cout << "dif: " << dif.norm() << std::endl;

        if (length >= radius)
        {
            std::cout << "notValid1" << std::endl;
            validGrasp = false;
            return;

        }

    }

    std::cout << "length: " << length << std::endl;
    std::cout << "radius: " << radius << std::endl;


    if(length + 10 >= radius && graspPreshape == "Precision Preshape")
    {
        std::cout << "Precisin not 1cm" << std::endl;
        validGrasp = true;
        return;
    }

    Eigen::Vector3f delta2 = approachDir * 10.f;
    ApproachMovementGenerator::updateEEFPose(delta2);

    Eigen::Vector4f p = getEEFPose().col(3);
//    std::cout << "\npose1: " << p.transpose() << std::endl;
    Eigen::Vector3f tmp(p[0], p[1], p[2]);

    Eigen::Vector3f dif = tmp - ps;
//    std::cout << "dif1: " << dif.norm() << std::endl;

    if (eef_cloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
    {
        std::cout << "collision" << std::endl;
        validGrasp = false;
        return;
    }
}

void ApproachMovementSkeleton::calculateApproachDirRound(PrincipalAxis3D &pca)
{
    Eigen::Vector3f a1 = pca.pca1;
    Eigen::Vector3f a2 = pca.pca1 * (-1);
    Eigen::Vector3f b1 = pca.pca2;
    Eigen::Vector3f b2 = pca.pca2 * (-1);
    approachDirs.push_back(a1);
    approachDirs.push_back(a2);
    approachDirs.push_back(b1);
    approachDirs.push_back(b2);

    Eigen::Vector3f a = Math::createMidVector(pca.pca1, pca.pca2);
    Eigen::Vector3f b = Math::createMidVector(pca.pca1  * (-1), pca.pca2);
    Eigen::Vector3f c = Math::createMidVector(pca.pca1  * (-1), pca.pca2 * (-1));
    Eigen::Vector3f d = Math::createMidVector(pca.pca1, pca.pca2 * (-1));
    approachDirs.push_back(a);
    approachDirs.push_back(b);
    approachDirs.push_back(c);
    approachDirs.push_back(d);
}

void ApproachMovementSkeleton::calculateApproachDirRectangular(PrincipalAxis3D &pca)
{
    approachDirs.push_back(pca.pca1);

    Eigen::Vector3f approach = pca.pca1 * (-1);
    approachDirs.push_back(approach);
}

void ApproachMovementSkeleton::calculateApproaches()
{
    float ratio = pca.eigenvalue1 / pca.eigenvalue2;
    std::cout << "ratio: " << ratio << std::endl;

    if (ratio < RATIO_THREASHOLD)
    {
        calculateApproachDirRound(pca);

    } else {
        calculateApproachDirRectangular(pca);
    }
}

string ApproachMovementSkeleton::getGraspPreshape()
{
    return graspPreshape;
}

SoSeparator* ApproachMovementSkeleton::getSep() {

    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(plane.p, 10.f, 0.f, 1.f, 0.f, 0.f));
    return sep;
}

RobotNodePtr ApproachMovementSkeleton::getTCP()
{
    return eef_cloned->getPreshape(graspPreshape)->getTCP();
}

bool ApproachMovementSkeleton::setEEFPose(const Eigen::Matrix4f& pose)
{

    if (!validGrasp)
    {
        return false;
    }

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
