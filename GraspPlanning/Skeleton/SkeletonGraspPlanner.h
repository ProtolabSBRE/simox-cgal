#ifndef __SKELETON_GRASP_PLANNER_H__
#define __SKELETON_GRASP_PLANNER_H__

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/GraspPlanner/GraspPlanner.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include "ApproachMovementSkeleton.h"
#include "SimoxCGAL.h"
#include "SegmentedObject.h"

#include <iostream>
#include <sstream>

namespace SimoxCGAL
{

struct GraspPlannerEvaluation
{
    GraspPlannerEvaluation()
    {
        nrGraspsGenerated = 0;
        nrGraspsValid = 0;
        nrGraspsInvalidCollision = 0;
        nrGraspsInvalidFC = 0;
        nrGraspsInvalidContacts = 0;
        nrGraspsValidPrecision = 0;
        nrGraspsValidPower = 0;
        fcCheck = false;
        minQuality = 0.0f;
    }

    void print()
    {
        std::cout << "---------------- GRASP PLANNER EVALUATION -----------" << std::endl;
        if (fcCheck)
            std::cout << "ForceClosure check: true" << std::endl;
        else
            std::cout << "ForceClosure check: false" << std::endl;
        std::cout << "Min Quality: " << minQuality << std::endl;

        std::cout << "nrGraspsGenerated:" << nrGraspsGenerated << std::endl;
        std::cout << "nrGraspsValid:" << nrGraspsValid << std::endl;
        std::cout << "nrGraspsInValid:" << nrGraspsInvalidCollision+nrGraspsInvalidFC+nrGraspsInvalidContacts << std::endl;
        std::cout << "nrGraspsValidPrecision:" << nrGraspsValidPrecision << std::endl;
        std::cout << "nrGraspsValidPower:" << nrGraspsValidPower << std::endl;
        std::cout << "nrGraspsInvalidCollision:" << nrGraspsInvalidCollision << std::endl;
        std::cout << "nrGraspsInvalidFC:" << nrGraspsInvalidFC << std::endl;
        std::cout << "nrGraspsInvalidContacts:" << nrGraspsInvalidContacts << std::endl;
        VR_ASSERT (timeGraspMS.size() == graspScore.size());
        VR_ASSERT (timeGraspMS.size() == graspValid.size());
        VR_ASSERT (timeGraspMS.size() == graspTypePower.size());
        float timeAcc = 0;
        float scoreAcc = 0;
        int nrGrasps = (int)timeGraspMS.size();
        int nrValid = 0;
        int nrPower = 0;
        int nrPrecision = 0;
        for (size_t i=0;i<timeGraspMS.size();i++)
        {
            timeAcc+=timeGraspMS.at(i);
            if (graspValid.at(i))
            {
                nrValid++;
                scoreAcc+=graspScore.at(i);
            }
            if (graspTypePower.at(i))
                nrPower++;
            else
                nrPrecision++;
        }
        std::cout << "Time complete: " << timeAcc << " ms" << std::endl;
        if (nrGrasps>0)
        {
            std::cout << "Avg time per grasp (valid&invalid):" << (float)timeAcc / (float)nrGrasps << " ms" << std::endl;
            float percPower = (float)nrPower / (float)nrGrasps;
            float percPrec = (float)nrPrecision / (float)nrGrasps;
            std::cout << "Precision grasps:" << nrPrecision << " -> " << percPrec*100 << "%" << std::endl;
            std::cout << "Power grasps:" << nrPower << " -> " << percPower*100 << "%" << std::endl;
        }
        if (nrValid>0)
        {
            std::cout << "Avg score (valid):" << scoreAcc / nrValid << std::endl;
        }
    }

    static std::string GetCSVHeader()
    {
        std::stringstream fs;
        fs << "minQuality" << ","
           << "nrGraspsGenerated" << ","
           << "nrGraspsValid" << ","
           << "nrGraspsInValid" << ","
           << "nrGraspsValidPrecision" << ","
           << "nrGraspsValidPower" << ","
           << "nrGraspsInvalidCollision" << ","
           << "nrGraspsInvalidFC" << ","
           << "nrGraspsInvalidContacts" << ","
           << "AverageDurationMS" << ","
           << "percPowerGrasps" << ","
           << "percPrecGraps" << ","
           << "avgScore";
        return fs.str();
    }

    std::string toCSVString() const
    {
        float timeAcc = 0;
        float scoreAcc = 0;
        int nrGrasps = (int)timeGraspMS.size();
        int nrValid = 0;
        int nrPower = 0;
        int nrPrecision = 0;
        for (size_t i=0;i<timeGraspMS.size();i++)
        {
            timeAcc+=timeGraspMS.at(i);
            if (graspValid.at(i))
            {
                nrValid++;
                scoreAcc+=graspScore.at(i);
            }
            if (graspTypePower.at(i))
                nrPower++;
            else
                nrPrecision++;
        }
        float percPower = 0;
        float percPrec = 0;
        if (nrGrasps>0)
        {
            percPower = (float)nrPower / (float)nrGrasps;
            percPrec = (float)nrPrecision / (float)nrGrasps;
        }
        float avgScore = 0;
        if (nrValid>0)
        {
            avgScore = scoreAcc / nrValid ;
        }


        std::stringstream fs;
        fs << minQuality << ","
           << nrGraspsGenerated << ","
           << nrGraspsValid << ","
           << (nrGraspsInvalidCollision+nrGraspsInvalidFC+nrGraspsInvalidContacts) << ","
           << nrGraspsValidPrecision << ","
           << nrGraspsValidPower << ","
           << nrGraspsInvalidCollision << ","
           << nrGraspsInvalidFC << ","
           << nrGraspsInvalidContacts << ","
           << (float)timeAcc / (float)nrGrasps << ","
           << percPower << ","
           << percPrec << ","
           << avgScore;
        return fs.str();
    }


    int nrGraspsGenerated;
    int nrGraspsValid;
    int nrGraspsInvalidCollision;
    int nrGraspsInvalidFC; // or quality
    int nrGraspsInvalidContacts;

    int nrGraspsValidPrecision;
    int nrGraspsValidPower;

    std::vector<float> timeGraspMS;     // time per grasp generation
    std::vector<float> graspScore;      //grasp quality
    std::vector<bool> graspValid;       //grasp valid
    std::vector<bool> graspTypePower;   //grasp type

    bool fcCheck;
    float minQuality;
};

    /*!
    *
    *
    * A general grasp planning class that utilizes ApprachMovementGenerator for generating grasp hypothesis and
    * GraspQualityMeasure to score them.
    *
    */
class SkeletonGraspPlanner : public GraspStudio::GraspPlanner
{
public:

    /*!
            Constructor
            \param graspSet All planned grasps are added to this GraspSet.
            \param graspQuality The quality of generated grasps is evaluated with this object
            \param approach Approach movements are generated by this object.
            \param minQuality The quality that must be achieved at minimum by the GraspQualityMesurement module
            \param forceClosure When true, only force closure grasps are generated.
        */
    SkeletonGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, ApproachMovementSkeletonPtr approach, float minQuality = 0.0f, bool forceClosure = true);

    // destructor
    virtual ~SkeletonGraspPlanner();

    /*!
            Creates new grasps.
            \param nrGrasps The number of grasps to be planned.
            \param timeOutMS The time out in milliseconds. Planning is stopped when this time is exceeded. Disabled when zero.
            \return Number of generated grasps.
        */
    virtual int plan(int nrGrasps, int timeOutMS = 0, VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr());


    void setParams(float minQuality, bool forceClosure);


    GraspPlannerEvaluation getEvaluation();
protected:

    bool timeout();

    VirtualRobot::GraspPtr planGrasp(VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr());

    VirtualRobot::SceneObjectPtr object;
    VirtualRobot::EndEffectorPtr eef;
    std::string preshape;

    VirtualRobot::EndEffector::ContactInfoVector contacts;
    GraspStudio::GraspQualityMeasurePtr graspQuality;
    ApproachMovementSkeletonPtr approach;

    GraspPlannerEvaluation eval;

    float minQuality;
    bool forceClosure;
    std::clock_t startTime;
    int timeOutMS;
};

typedef boost::shared_ptr<SkeletonGraspPlanner> SkeletonGraspPlannerPtr;

}
#endif /* __GENERIC_GRASP_PLANNER_H__ */
