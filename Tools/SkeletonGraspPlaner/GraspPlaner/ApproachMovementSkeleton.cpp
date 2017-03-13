#include "ApproachMovementSkeleton.h"

#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include  "VirtualRobot/VirtualRobotImportExport.h"
#include "VirtualRobot/RobotConfig.h"
#include "VirtualRobot/MathTools.h"
#include "Eigen/SVD"

#include "Inventor/nodes/SoMatrixTransform.h"
#include "Inventor/nodes/SoTransform.h"
#include "Inventor/nodes/SoUnits.h"

#include "Math.h"

#define RATIO_THREASHOLD 1.3f

using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;


ApproachMovementSkeleton::ApproachMovementSkeleton(VirtualRobot::SceneObjectPtr object, SkeletonPtr skeleton, SurfaceMeshPtr mesh, SegmentedObjectPtr segmentation, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape)
    : ApproachMovementGenerator(object, eef, graspPreshape), decider(new DeciderGraspPreshape), skeleton(skeleton), segmentation(segmentation), mesh(mesh)
{
    name = "ApproachMovementSkeleton";
    sep = new SoSeparator;
    sep->ref();
    this->graspPreshape = graspPreshape;
    powerGrasp = false;
    precisionGrasp = false;
    validGrasp = true;

    bool index = false;

    if (!eef->hasPreshape(graspPreshape))
    {
        useAllGrasps = true;

    } else {
        useAllGrasps = false;
    }

    currentSubpart = 0;
    std::cout << "Segment size: " << segmentation->getObjectParts().size() << std::endl;


    while (!index)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

//        std::cout << "Segment: " << segmentation->getObjectParts().at(currentSubpart)->name << std::endl;

        if (currentSubpart >= segmentation->getObjectParts().size())
        {
            std::cout << "NO SEGMENTS\n" << std::endl;
            break;
        }

        if (subpart->palpable)
        {
            index = true;
            currentSkeletonVertex = 0;
            std::cout << "currentSubpart: " << currentSubpart << std::endl;
            std::cout << "currentIndex: " << currentSkeletonVertex << std::endl;

        } else
        {
            currentSubpart++;

        }
    }

    noMoreGrasps = false;

    std::cout << "Init im Konstruktor fertig." << std::endl;
}

ApproachMovementSkeleton::~ApproachMovementSkeleton()
{

}

Eigen::Matrix4f ApproachMovementSkeleton::createNewApproachPose()
{
    std::cout << "createNewApproachPose" << std::endl;
    Eigen::Matrix4f pose = getEEFPose();
    openHand();
    Eigen::Vector3f position;
    Eigen::Vector3f approachDir;

    position = graspsPoint.plane.p;
    approachDir = graspsPoint.approachDirs.back();
    graspsPoint.approachDirs.pop_back();

    std::cout << "remaining: " << graspsPoint.approachDirs.size() << std::endl;

    this->aporachDirGlobal = approachDir;
    eef_cloned->setPreshape(graspsPoint.decideGraspPreshape);

    // set new pose
    setEEFToApproachPose(position,approachDir);

    // move away until valid
    moveEEFAway(approachDir, 0.5f);

    Eigen::Matrix4f poseB = getEEFPose();


    // restore original pose
//    eefRobot->setGlobalPoseForRobotNode(eef_cloned->getPreshape(graspsPoint.decideGraspPreshape)->getTCP(), pose);
    setEEFPose(pose);

    std::cout << "createApproach done." << std::endl;
    return poseB;

}

void ApproachMovementSkeleton::next()
{

    float dist = 0.f;

    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

    if (currentSkeletonVertex <= (subpart->sortedSkeletonPartIndex.size() - 1))
    {
        currentSkeletonVertex = distSkeleton(dist);
        std::cout << "\napproachDir->nextSubpart: " << currentSubpart << "//" << segmentation->getObjectParts().size() << std::endl;
        std::cout << "approachDir->nextInterval: " << currentSkeletonVertex << "\n\n";
        return;
    }

    currentSubpart++;
    currentSkeletonVertex = 0;

    std::cout << "\n2approachDir->nextSubpart: " << currentSubpart << "//" << segmentation->getObjectParts().size() << std::endl;
    std::cout << "2approachDir->nextInterval: " << currentSkeletonVertex << "\n\n";

//    if (nextVertex <= (segmentation->getMembers().at(currentSubpart)->sortedSkeletonPartIndex.size() - 1))
//    {
//        currentSkeletonVertex = nextVertex;

//    } else if (nextSubpart <= (segmentation->getMembers().size() - 1))
//    {

//        currentSubpart++;

//        while (!segmentation->getMembers().at(currentSubpart)->palpable)
//        {
//            currentSubpart++;
//        }

//        currentSkeletonVertex = 0;

//    } else {
//        currentSubpart++;
//    }


    std::cout << "\napproachDir->nextSubpart: " << currentSubpart << "//" << segmentation->getObjectParts().size() << std::endl;
    std::cout << "approachDir->nextInterval: " << currentSkeletonVertex << "\n\n";

}

int ApproachMovementSkeleton::distSkeleton(float &dist)
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
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

    int sizeSubparts = segmentation->getObjectParts().size() - 1;

    if (currentSubpart <= sizeSubparts)
    {
        if (currentSkeletonVertex <= (subpart->sortedSkeletonPartIndex.size() - 1))
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

bool ApproachMovementSkeleton::areMoreValidGrasps()
{
    return (!(graspsPoint.approachDirs.size() == 0));
}

bool ApproachMovementSkeleton::getApproachDirection()
{
    SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(segmentation->getObjectParts().at(currentSubpart));

    std::cout << "palpalbe: " << subpart->palpable << std::endl;

    if (!subpart->palpable)
    {
        currentSkeletonVertex = subpart->sortedSkeletonPartIndex.size();
        return false;
    }

    std::cout << "currentSubpart: " << currentSubpart << " of " << segmentation->getObjectParts().size() - 1 << std::endl;
    std::cout << "powerGrasp: " << powerGrasp << std::endl;
    std::cout << "precisionGrasp: " << precisionGrasp << std::endl;
    std::cout << "currentSkeletonVertexIndex: " << currentSkeletonVertex << std::endl;


    //calculate Intervall
    bool found_1 = false;
    std::vector<SkeletonVertex> interval_precision;
    float length_precision = 47.f;
    bool precisionValid = false;
    Diameter r_precision;
    MathTools::Plane plane_p;
    vector<Eigen::Vector3f> v_pointsPrecision;
    PrincipalAxis3D pca1;

    bool found_2 = false;
    std::vector<SkeletonVertex> interval_power;
    float length_power = 94.f;
    bool powerValid = false;
    Diameter r_power;
    MathTools::Plane plane_k;
    vector<Eigen::Vector3f> v_pointsPower;
    PrincipalAxis3D pca2;

    if (precisionGrasp)
    {

        found_1 = subpart->calculateInterval(skeleton, currentSkeletonVertex, length_precision, interval_precision);

        if (found_1) {

            std::cout << "interval_precision.size: " << interval_precision.size() << std::endl;
            SkeletonPointPtr point = subpart->skeletonPart[subpart->sortedSkeletonPartIndex.at(currentSkeletonVertex)];
            Point pos = (*skeleton)[interval_precision.at(0)].point;
            Eigen::Vector3f posE(pos[0], pos[1], pos[2]);

            plane_p.p = posE;

            Point n1 = (*skeleton)[point->neighbor.front()].point;
            Point n2 = (*skeleton)[point->neighbor.back()].point;

            Eigen::Vector3f n1e(n1[0], n1[1], n1[2]);
            Eigen::Vector3f n2e(n2[0], n2[1], n2[2]);


            Math::calculateApproachPlane(plane_p.p, n1e, n2e, plane_p.n, sep);

            r_precision = Math::getPlanesWithMeshPoints(skeleton, mesh, interval_precision, plane_p, v_pointsPrecision);
            r_precision.print();
            pca1 = Math::calculatePCA(v_pointsPrecision, sep);
    //        r_precision = decider->calculateDiameter(v_planePrecision, v_pointsPrecision);
            float rad = pca1.t2;
            precisionValid = decider->decidePrecisionPreshape(rad);

            graspsPoint.decideGraspPreshape = graspPreshape;
            graspsPoint.plane = plane_p;
    //        r = Math::projectPointsToPlane(v_planePrecision, v_pointsPrecision);
        }


    }

    if (powerGrasp)
    {

        found_2 = subpart->calculateInterval(skeleton, currentSkeletonVertex, length_power, interval_power);

        if (found_2) {

            std::cout << "interval_power.size: " << interval_power.size() << std::endl;

            SkeletonPointPtr point = subpart->skeletonPart[subpart->sortedSkeletonPartIndex.at(currentSkeletonVertex)];
            Point pos = (*skeleton)[interval_power.at(0)].point;
            Eigen::Vector3f posE(pos[0], pos[1], pos[2]);

            plane_k.p = posE;

            Point n1 = (*skeleton)[point->neighbor.front()].point;
            Point n2 = (*skeleton)[point->neighbor.back()].point;

            Eigen::Vector3f n1e(n1[0], n1[1], n1[2]);
            Eigen::Vector3f n2e(n2[0], n2[1], n2[2]);


            Math::calculateApproachPlane(plane_k.p, n1e, n2e, plane_k.n, sep);


            r_power = Math::getPlanesWithMeshPoints(skeleton, mesh, interval_power, plane_k, v_pointsPower);
            r_power.print();
    //        r_power = decider->calculateDiameter(v_planePower, v_pointsPower);
            pca2 = Math::calculatePCA(v_pointsPrecision, sep);
            float rad = pca2.t2;
            powerValid = decider->decidePowerPreshape(rad);
            graspsPoint.decideGraspPreshape = "Power Preshape";
            graspsPoint.plane = plane_k;
    //        r = Math::projectPointsToPlane(v_planePower, v_pointsPower);
        }


    }

//    if (!found1 && !found2)
//    {
//        std::cout << "kein Intervall gültig" << std::endl;
//    }

    PrincipalAxis3D pca;

    if(useAllGrasps)
    {
        if (precisionValid)
        {
            std::cout << "validPrecison" << std::endl;
//            currentSubpart = afterPrecision_Subpart;
//            currentSkeletonVertex = afterPrecision_Vertex;
            graspsPoint.decideGraspPreshape = "Precision Preshape";
            graspsPoint.plane = plane_p;
            pca = pca1;

            precisionInterval.plane = plane_p;
            precisionInterval.points = v_pointsPrecision;


//            r = Math::projectPointsToPlane(v_planePrecision, v_pointsPrecision);

        } else if (powerValid){

            std::cout << "powerValid" << std::endl;
            powerInterval.plane = plane_k;
            powerInterval.points = v_pointsPower;

            graspsPoint.decideGraspPreshape = "Power Preshape";
            pca = pca2;
//            r = Math::projectPointsToPlane(v_planePower, v_pointsPower);

        } else {

            noMoreGrasps = false;
            return false;
        }

    }

//    precisionInterval.planes = v_planePrecision;
//    powerInterval.planes = v_planePower;

    std::cout << "graspPoint//preshape: " << graspsPoint.decideGraspPreshape << std::endl;


    //berechne PCA
//    PrincipalAxis3D pca = Math::calculatePCA(r, sep);
    this->pca = pca;
    float ratio = pca.eigenvalue1 / pca.eigenvalue2;
    std::cout << "ratio: " << ratio << std::endl;


//    float ev1 =(pca.eigenvalue1 * pca.eigenvalue1) / (points.size() - 1);
//    float ev2 =(pca.eigenvalue2 * pca.eigenvalue2) / (points.size() - 1);
//    float ev3 =(pca.eigenvalue3 * pca.eigenvalue3) / (points.size() - 1);
//    cout << "ev1: " << ev1 << endl;
//    cout << "ev2: " << ev2 << endl;
//    cout << "ev3: " << ev3 << endl;

//    Eigen::Vector3f t = pca.pca1 * std::sqrt(ev1);
//    Eigen::Vector3f x = pca.pca2 * std::sqrt(ev2);


    SoSeparator* res = new SoSeparator;
    res->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    res->addChild(u);

    SoSeparator* a = new SoSeparator;
    SoSeparator* b = new SoSeparator;
    SoSeparator* c = new SoSeparator;

    SoTransform* trans = new SoTransform;
    SbMatrix mt;
    SbVec3f vec(graspsPoint.plane.p[0],graspsPoint.plane.p[1], graspsPoint.plane.p[2]);
    mt.setTranslate(vec);
    trans->setMatrix(mt);

//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(graspsPoint.plane.p, 2.f, 0.f, 0.f, 1.f, 1.f));

//    for (int i = 0; i < interval_precision.size(); i++)
//    {
//        Point a = skeleton[interval_precision.at(i)].point;
//        Eigen::Vector3f p(a[0], a[1], a[2]);
//        sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(p, graspsPoint.plane.n, 30.f, 0.3f, false, 0.f ,1.f, 1.f));


//    }


//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(pca.pca1 * pca.eigenvalue1, 3.5, 0.f, 1.f, 0.f, 0.f));
//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(pca.pca2 * pca.eigenvalue2, 3.5, 0.f, 1.f, 0.f, 0.f));

    Eigen::Vector3f p1 = pca.pca1 * pca.t1;
    Eigen::Vector3f p2 = pca.pca2 * pca.t2;

    std::cout << "p1: " << p1.norm() << std::endl;
    std::cout << "p2: " << p2.norm() << std::endl;

    a->addChild(trans);
    a->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(pca.pca1, pca.t1, 0.5f, VirtualRobot::VisualizationFactory::Color::Red(0.5f)));
    res->addChild(a);

    b->addChild(trans);
    b->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(pca.pca2, pca.t2, 0.5f, VirtualRobot::VisualizationFactory::Color::Green(0.5f)));
    res->addChild(b);
    c->addChild(trans);
    c->addChild(VirtualRobot::CoinVisualizationFactory::CreateArrow(pca.pca3, 5.f, 2.f, VirtualRobot::VisualizationFactory::Color::Blue(0.5f)));
//    res->addChild(c);

//    sep->addChild(res);


//    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(graspsPoint.plane.p, graspsPoint.plane.n, 10.f, 0.5f, false, 0.f, 0.f, 1.f));
    //    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(pos, d2, 5.f, 0.5f, false, 0.f, 0.f, 1.f));
    //    sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(pos, result, 10.f, 0.5f, false, 1.f, 0.f, 0.f));



    if (ratio < RATIO_THREASHOLD)
    {
        calculateApproachDirRound(pca);

    } else {
        calculateApproachDirRectangular(pca);
    }

    return true;
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

    Eigen::Vector3f y = graspsPoint.plane.n;
    Eigen::Vector3f x;
    x = y.cross(z);
    x.normalize();


    poseFinal.block(0, 0, 3, 1) = x;
    poseFinal.block(0, 1, 3, 1) = y;
    poseFinal.block(0, 2, 3, 1) = z;


    validGrasp = true;

    //set GCP manually
//    eefRobot->setGlobalPoseForRobotNode(eef_cloned->getPreshape(graspsPoint.decideGraspPreshape)->getTCP(), poseFinal);
//    setEEFPose(poseFinal);
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
        updateEEFPose(delta);

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


    if(length + 10 >= radius && graspsPoint.decideGraspPreshape == "Precision Preshape")
    {
        std::cout << "Precisin not 1cm" << std::endl;
        validGrasp = true;
        return;
    }

    Eigen::Vector3f delta2 = approachDir * 10.f;
    updateEEFPose(delta2);

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
//    std::cout << "Round before// " << graspsPoint.approachDirs.size() << " points.\n";

    Eigen::Vector3f a1 = pca.pca1;
    Eigen::Vector3f a2 = pca.pca1 * (-1);
    Eigen::Vector3f b1 = pca.pca2;
    Eigen::Vector3f b2 = pca.pca2 * (-1);
    graspsPoint.approachDirs.push_back(a1);
    graspsPoint.approachDirs.push_back(a2);
    graspsPoint.approachDirs.push_back(b1);
    graspsPoint.approachDirs.push_back(b2);

    Eigen::Vector3f a = Math::createMidVector(pca.pca1, pca.pca2);
    Eigen::Vector3f b = Math::createMidVector(pca.pca1  * (-1), pca.pca2);
    Eigen::Vector3f c = Math::createMidVector(pca.pca1  * (-1), pca.pca2 * (-1));
    Eigen::Vector3f d = Math::createMidVector(pca.pca1, pca.pca2 * (-1));
    graspsPoint.approachDirs.push_back(a);
    graspsPoint.approachDirs.push_back(b);
    graspsPoint.approachDirs.push_back(c);
    graspsPoint.approachDirs.push_back(d);

    std::cout << "Round//Current size: " << graspsPoint.approachDirs.size() << std::endl;

}

void ApproachMovementSkeleton::calculateApproachDirRectangular(PrincipalAxis3D &pca)
{
    graspsPoint.approachDirs.push_back(pca.pca1);

    Eigen::Vector3f approach = pca.pca1 * (-1);
    graspsPoint.approachDirs.push_back(approach);

    std::cout << "Rectangular//Current size: " << graspsPoint.approachDirs.size() << std::endl;

}

string ApproachMovementSkeleton::getGraspPreshape()
{
    return graspsPoint.decideGraspPreshape;
}

SoSeparator* ApproachMovementSkeleton::getSep() {

    if (graspsPoint.decideGraspPreshape == "Precision Preshape" )
    {

//        sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(graspsPoint.plane.p, graspsPoint.plane.n, 40.f, 0.7f, false, 0.f ,1.f, 1.f));

        for (int i = 0; i < precisionInterval.points.size(); i++)
        {
//            sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(precisionInterval.points.at(i), 2.f, 0.f, 1.f, 1.f, 0.f));
        }

        std::vector<SkeletonVertex> var = precisionInterval.interval;

//        for (int i = 0; i < var.size(); i++)
//        {
//            Skeleton_vertex vertex = var.at(i);
//            Point point = skeleton[vertex];
//            Eigen::Vector3f pp(point[0], point[1], point[2]);
//            sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(pp, 2.f, 0.f, 1.f, 0.f, 0.f));


//        }

    }


    if (graspsPoint.decideGraspPreshape == "Power Preshape" )
    {

//        sep->addChild(VirtualRobot::CoinVisualizationFactory::CreatePlaneVisualization(powerInterval.plane.p, powerInterval.plane.n, 0.5f, 0.f, false, 0.f ,1.f, 1.f));

        for (int i = 0; i < powerInterval.points.size(); i++)
        {
//            sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(powerInterval.points.at(i), 2.f, 0.f, 1.f, 1.f, 0.f));
        }


//        for (int i = 0; i < powerInterval.interval.size(); i++)
//        {
//            Skeleton_vertex vertex = powerInterval.interval.at(i);
//            Point point = skeleton[vertex];
//            Eigen::Vector3f pp(point[0], point[1], point[2]);
//            sep->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(pp, 2.f, 0.f, 1.f, 0.f, 0.f));


//        }

    }

    return sep;
}

RobotNodePtr ApproachMovementSkeleton::getTCP()
{
//    return eef_cloned->getPreshape(graspPreshape)->getTCP();
}

void ApproachMovementSkeleton::setPowerGrasp(bool state)
{
    powerGrasp = state;
}

void ApproachMovementSkeleton::setPrecisionGrasp(bool state)
{
    precisionGrasp = state;
}

void ApproachMovementSkeleton::clear() {
    sep->removeAllChildren();
}

bool ApproachMovementSkeleton::setEEFPose(const Eigen::Matrix4f& pose)
{

    if (!validGrasp)
    {
        return false;
    }

//    eefRobot->setGlobalPoseForRobotNode(eef_cloned->getPreshape(graspsPoint.decideGraspPreshape)->getTCP(), pose);
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

//     return eef_cloned->getPreshape(graspsPoint.decideGraspPreshape)->getTCP()->getGlobalPose();
}

VirtualRobot::RobotPtr ApproachMovementSkeleton::getEEFRobotClone()
{
    return eefRobot;
}

bool ApproachMovementSkeleton::done()
{
    return noMoreGrasps;
}
