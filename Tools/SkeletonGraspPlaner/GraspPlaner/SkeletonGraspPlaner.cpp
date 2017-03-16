#include "SkeletonGraspPlaner.h"


#include "VirtualRobot/RobotConfig.h"

using namespace std;
using namespace VirtualRobot;

SkeletonGraspPlanner::SkeletonGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, ApproachMovementSkeletonPtr approach, float minQuality, bool forceClosure)
    : GraspPlanner(graspSet), graspQuality(graspQuality), minQuality(minQuality), forceClosure(forceClosure)
{
    this->approach = approach;
    eef = approach->getEEF();
    object = graspQuality->getObject();
    verbose = true;
}

SkeletonGraspPlanner::~SkeletonGraspPlanner()
{

}

int SkeletonGraspPlanner::plan(int nrGrasps, int timeOutMS, SceneObjectSetPtr obstacles)
{
    startTime = clock();
    this->timeOutMS = timeOutMS;

    int nLoop = 0;
    int nGraspsCreated = 0;

    if (verbose)
    {
        GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF: '" << approach->getEEF()->getName() << "' and object: '" << graspQuality->getObject()->getName() << "'.\n";
        GRASPSTUDIO_INFO << ": Approach movements are generated with: " << approach->getName() << endl;
        GRASPSTUDIO_INFO << ": Approach movements are generated with preshape: '" << approach->getGraspPreshape() <<  "'\n";
        GRASPSTUDIO_INFO << ": Grasps are evaluated with: " << graspQuality->getName() << endl;
    }

    bool valid = true;
    bool a = true;
    bool b = true;

    while (!timeout() && nGraspsCreated < nrGrasps && valid)
    {
        if (!approach->areMoreSegments())
        {
            GRASPSTUDIO_INFO << ": All grasps generated." << endl;
            break;
        }

        if (approach->getApproachesNumber() == 0)
        {
            cout << "PRÜFE" << endl;
            a = approach->setNextIndex();

            if (!a)
            {
                cout << "ENDE" << endl;
                break;
            }

            b = approach->calculateApproachDirection();

            if (!b)
            {
                continue;
            }
        }

        //generiere Griff
        GraspPtr g = planGrasp(obstacles);

        if (g)
        {
            if (graspSet)
            {
                graspSet->addGrasp(g);
            }

            plannedGrasps.push_back(g);
            nGraspsCreated++;
        }

        nLoop++;

        std::cout << "DONE\n\n" << std::endl;
    }

    if (verbose)
    {
        GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops" << endl;
    }

    return nGraspsCreated;
}

GraspPtr SkeletonGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
{
    if (!approach->isValid())
    {
        return GraspPtr();
    }

    cout << "planGrasp" << endl;
    string sGraspPlanner("Simox - GraspStudio - ");
    sGraspPlanner += graspQuality->getName();
    string sGraspNameBase = "Grasp ";

    RobotPtr robot = approach->getEEFOriginal()->getRobot();
    RobotNodePtr tcp = eef->getTcp();

    VR_ASSERT(robot);
    VR_ASSERT(tcp);

    Eigen::Matrix4f p = approach->createNewApproachPose();
    bool bRes = approach->setEEFPose(p);

    eef->setPreshape(approach->getGraspPreshape());
    contacts = eef->closeActors(object);
    eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());

    if (contacts.size() < 2)
    {
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
        }

        return GraspPtr();
    }

    graspQuality->setContactPoints(contacts);
    float score = graspQuality->getGraspQuality();

    if (score < minQuality)
    {
        return GraspPtr();
    }

    if (forceClosure && !graspQuality->isGraspForceClosure())
    {
        return GraspPtr();
    }

    // found valid grasp
    if (verbose)
    {
        GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << endl;
    }

    stringstream ss;
    ss << sGraspNameBase << (graspSet->getSize() + 1);
    string sGraspName = ss.str();
    Eigen::Matrix4f objP = object->getGlobalPose();
    Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
    GraspPtr g(new Grasp(sGraspName, robot->getType(), eef->getName(), pLocal, sGraspPlanner, score, approach->getGraspPreshape()));
    // set joint config
    RobotConfigPtr config = eef->getConfiguration();
    map<string, float> configValues = config->getRobotNodeJointValueMap();
    g->setConfiguration(configValues);
    return g;

}

bool SkeletonGraspPlanner::timeout()
{
    if (timeOutMS <= 0)
    {
        return false;
    }

    std::clock_t endTime = clock();
    int timeMS = (int) (((float)(endTime - startTime) / (float)CLOCKS_PER_SEC) * 1000.0);
    return (timeMS > timeOutMS);
}


