#include "SkeletonGraspPlaner.h"


#include "VirtualRobot/RobotConfig.h"

using namespace std;

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

int SkeletonGraspPlanner::plan(int nrGrasps, int timeOutMS, VirtualRobot::SceneObjectSetPtr obstacles)
{
//    startTime = clock();
//    this->timeOutMS = timeOutMS;

    int nLoop = 0;
    int nGraspsCreated = 0;

    if (verbose)
    {
        GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF:" << approach->getEEF()->getName() << " and object:" << graspQuality->getObject()->getName() << ".\n";
        GRASPSTUDIO_INFO << ": Approach movements are generated with " << approach->getName() << endl;
        GRASPSTUDIO_INFO << ": Approach movements are generated with preshape '" << approach->getGraspPreshape() <<  "'\n";
        GRASPSTUDIO_INFO << ": Grasps are evaluated with " << graspQuality->getName() << endl;
    }

    bool valid = approach->isValid();

    if (!approach->areMoreValidGrasps())
    {
         bool validShape = false;

         while (valid)
         {
             validShape = approach->getApproachDirection();

             if (!validShape)
             {
                 std::cout << "nicht gültig" << std::endl;
                 approach->next();
                 valid = approach->isValid();


                 if (!valid)
                 {
                     if (approach->areMoreSegments())
                     {
                         approach->next();
                         valid = approach->isValid();
                         continue;
                     } else {
                         //keine Segmente!
                         valid = false;
                         break;
                     }
                 }

             } else {

                 break;
             }
         }

    }

    if ((!valid) && verbose)
    {
        GRASPSTUDIO_INFO << ": No Skeleton points for grasping left. All possible grasps are generated.\n";
        return nGraspsCreated;
    }


    while (/*!timeout() && */nGraspsCreated < nrGrasps && valid)
    {
        VirtualRobot::GraspPtr g = planGrasp(obstacles);

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

        if (!approach->areMoreValidGrasps())
        {
            while(valid)
            {
                approach->next();
                valid = approach->isValid();

                if (!valid)
                {
                    if (approach->areMoreSegments())
                    {
                        valid = true;
                        continue;
                    } else {
                        //keine Segmente mehr!
                        break;
                    }
                }

                bool tmp = approach->getApproachDirection();

                if (tmp)
                {
                    std::cout << "gültiges Intervall gefunden" << std::endl;
                    break;
                }
            }
        } else {

            std::cout << "moreGrasps" << std::endl;

        }
    }

    if (verbose)
    {
        GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops" << endl;
    }

    return nGraspsCreated;

}

VirtualRobot::GraspPtr SkeletonGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
{

    std::cout << "planGrasp" << std::endl;
    std::string sGraspPlanner("Simox - GraspStudio - ");
    sGraspPlanner += graspQuality->getName();
    std::string sGraspNameBase = "Grasp ";

    VirtualRobot::RobotPtr robot = approach->getEEFOriginal()->getRobot();
    VirtualRobot::RobotNodePtr tcp = eef->getTcp();

    VR_ASSERT(robot);
    VR_ASSERT(tcp);

    Eigen::Matrix4f p = approach->createNewApproachPose();
    bool bRes = approach->setEEFPose(p);

    if (!bRes)
    {
        std::cout << "nicht valid" << std::endl;
        return VirtualRobot::GraspPtr();
    }


    eef->setPreshape(approach->getGraspPreshape());
    contacts = eef->closeActors(object);
    eef->addStaticPartContacts(object, contacts, approach->getApproachDirGlobal());

    if (contacts.size() < 2)
    {
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
        }

        return VirtualRobot::GraspPtr();
    }

    graspQuality->setContactPoints(contacts);
    float score = graspQuality->getGraspQuality();

    if (score < minQuality)
    {
        return VirtualRobot::GraspPtr();
    }

    if (forceClosure && !graspQuality->isGraspForceClosure())
    {
        return VirtualRobot::GraspPtr();
    }

    // found valid grasp
    if (verbose)
    {
        GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << endl;
    }

    std::stringstream ss;
    ss << sGraspNameBase << (graspSet->getSize() + 1);
    std::string sGraspName = ss.str();
    Eigen::Matrix4f objP = object->getGlobalPose();
    Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
    VirtualRobot::GraspPtr g(new VirtualRobot::Grasp(sGraspName, robot->getType(), eef->getName(), pLocal, sGraspPlanner, score, approach->getGraspPreshape()));
    // set joint config
    VirtualRobot::RobotConfigPtr config = eef->getConfiguration();
    std::map< std::string, float> configValues = config->getRobotNodeJointValueMap();
    g->setConfiguration(configValues);
    return g;

}
