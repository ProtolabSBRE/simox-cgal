#include "SkeletonGraspPlanner.h"


#include "VirtualRobot/RobotConfig.h"

using namespace std;
using namespace VirtualRobot;

namespace SimoxCGAL
{

SkeletonGraspPlanner::SkeletonGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, ApproachMovementSkeletonPtr approach, float minQuality, bool forceClosure)
    : GraspPlanner(graspSet), graspQuality(graspQuality), minQuality(minQuality), forceClosure(forceClosure)
{
    this->approach = approach;
    eef = approach->getEEF();
    object = graspQuality->getObject();
    verbose = true;
    eval.fcCheck = forceClosure;
    eval.minQuality = minQuality;
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
        GRASPSTUDIO_INFO << ": Grasps are evaluated with: " << graspQuality->getName() << endl;
    }

    bool valid = true;
    bool a = true;
    bool b = true;

    while (!timeout() && nGraspsCreated < nrGrasps && valid)
    {
        if (!approach->moreSegmentsAvailable())
        {
            if (verbose)
                GRASPSTUDIO_INFO << ": All grasps generated." << endl;
            break;
        }

        if (verbose)
            VR_INFO << "Next approach pose..." << endl;

        if (approach->getApproachesNumber() == 0)
        {
            if (verbose)
                VR_INFO << "Generating new approach poses..." << endl;

            a = approach->setNextIndex();

            if (!a)
            {
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
    }

    if (verbose)
    {
        GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops" << endl;
    }

    return nGraspsCreated;
}

void SkeletonGraspPlanner::setParams(float minQuality, bool forceClosure)
{
    this->minQuality = minQuality;
    this->forceClosure = forceClosure;
    eval.fcCheck = forceClosure;
    eval.minQuality = minQuality;
}

GraspPlannerEvaluation SkeletonGraspPlanner::getEvaluation()
{
    return eval;
}

GraspPtr SkeletonGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
{
    auto start_time = chrono::high_resolution_clock::now();
    if (!approach->isValid())
    {
        return GraspPtr();
    }
    if (obstacles)
    {
        VR_WARNING << "Obstacles are not considered yet..." << endl;
    }

    string sGraspPlanner("Simox - GraspStudio - ");
    sGraspPlanner += graspQuality->getName();
    string sGraspNameBase = "Grasp ";

    RobotPtr robot = approach->getEEFOriginal()->getRobot();
    RobotNodePtr tcp = eef->getTcp();

    VR_ASSERT(robot);
    VR_ASSERT(tcp);

    Eigen::Matrix4f p;
    bool ok = approach->createNewApproachPose(p);

    // basic eval data
    bool powerG = (approach->getCurrentGraspType() == ApproachMovementSkeleton::PlanningParameters::Power);
    eval.graspTypePower.push_back(powerG);
    eval.nrGraspsGenerated++;

    if (!ok)
    {
        if (verbose)
            GRASPSTUDIO_INFO << "Could not build eef approach pose" << endl;
        auto end_time = chrono::high_resolution_clock::now();
        float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
        eval.graspScore.push_back(0.0f);
        eval.graspValid.push_back(false);
        eval.nrGraspsInvalidCollision++;
        eval.timeGraspMS.push_back(ms);
        return GraspPtr();
    }
    //bool bRes =
    approach->setEEFPose(p);

    eef->setPreshape(approach->getGraspPreshape());
    contacts = eef->closeActors(object);
    eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());



    if (contacts.size() < 2)
    {
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
        }
        auto end_time = chrono::high_resolution_clock::now();
        float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
        eval.graspScore.push_back(0.0f);
        eval.graspValid.push_back(false);
        eval.nrGraspsInvalidContacts++;
        eval.timeGraspMS.push_back(ms);

        return GraspPtr();
    }

    graspQuality->setContactPoints(contacts);
    float score = graspQuality->getGraspQuality();

    if (forceClosure && !graspQuality->isGraspForceClosure())
    {
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, not FC: " << endl;
        }
        auto end_time = chrono::high_resolution_clock::now();
        float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
        eval.graspScore.push_back(0.0f);
        eval.graspValid.push_back(false);
        eval.nrGraspsInvalidFC++;
        eval.timeGraspMS.push_back(ms);
        return GraspPtr();
    }

    if (score < minQuality)
    {
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, grasp score too low: " << score << endl;
        }
        auto end_time = chrono::high_resolution_clock::now();
        float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
        eval.graspScore.push_back(score);
        eval.graspValid.push_back(false);
        eval.nrGraspsInvalidFC++;
        eval.timeGraspMS.push_back(ms);
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

    auto end_time = chrono::high_resolution_clock::now();
    float ms = float(chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count());
    eval.graspScore.push_back(score);
    eval.graspValid.push_back(true);
    eval.nrGraspsValid++;
    if (powerG)
        eval.nrGraspsValidPower++;
    else
        eval.nrGraspsValidPrecision++;
    eval.timeGraspMS.push_back(ms);

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

}
