#ifndef __APPROACH_MOVEMENT_SKELETON_H__
#define __APPROACH_MOVEMENT_SKELETON_H__

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/ApproachMovementGenerator.h>
#include <VirtualRobot/SceneObject.h>
#include <vector>
#include "VirtualRobot/Nodes/RobotNode.h"

#include "SimoxCGAL.h"
#include "SegmentedObject.h"
#include "Math.h"
#include "DeciderGraspPreshape.h"
#include "Segmentation/Skeleton/SkeletonPoint.h"

#define POWER_GRASP "Power Preshape"
#define POWER_INTERVALL 94.f

#define PRECISION_GRASP "Precision Preshape"
#define PRECISION_INTERVALL 47.f

#define RATIO_THREASHOLD 1.3f

#define SAMPLING_LENGTH 0.f


//!
//! \brief DirAndPos
//! Speichert Greifposition und Anrückrichtung
//!
typedef std::pair<Eigen::Vector3f, Eigen::Vector3f> DirAndPos;
typedef std::map<SimoxCGAL::SkeletonVertex,SimoxCGAL::SkeletonPointPtr>::iterator SkeletonVertexIterator;

    /*!
    *
    *
    * This class generates grasping configs by setting the position from the skeleton and setting the EEF to a aligned position (calculated via PCA).
    * The remaining free DoF (the rotation around the normal) is set via interval (definite in via Skeleton).
    * Then the EEF is moved along the normal until a collision is detected or the GCP hits the object.
    * If needed, the EEF is moved back until a collision-free pose is found.
    *
    * Internally the EEF is cloned.
    *
    */
class ApproachMovementSkeleton : public GraspStudio::ApproachMovementGenerator
{
public:
    //        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//    struct GraspPoint{
//        VirtualRobot::MathTools::Plane plane;
//        std::vector<Eigen::Vector3f> approachDirs;
//        std::string decideGraspPreshape;
//    };

    /*!
            To generate approach movements an object and an end effector has to be specified.
            \param object The object.
            \param eef The end effector.
            \param graspPreshape An optional preshape that can be used in order to "open" the eef.
            \param maxRandDist If >0, the resulting apporach pose is randomly moved in the approach direction (away from the object) in order to create different distances to the object.
        */
    ApproachMovementSkeleton(VirtualRobot::SceneObjectPtr object, SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, SimoxCGAL::SegmentedObjectPtr segmentation, VirtualRobot::EndEffectorPtr eef, const std::string& graspPreshape = "");
    //! destructor
    ~ApproachMovementSkeleton();

    //! Creates a new pose for approaching and set it
    Eigen::Matrix4f createNewApproachPose();

    //! Return approach direction and position on skeleton
//    bool getApproachDirection(Eigen::Vector3f &storePos, Eigen::Vector3f& storeApproachDir);
    bool calculateApproachDirection();


    //! Sets EEF to a position so that the Z component of the GCP coord system is aligned with -approachDir
    bool setEEFToApproachPose(const Eigen::Vector3f& position, const Eigen::Vector3f& approachDir);

    void moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops = 1000);

    std::string getGraspPreshape();

    void next();

    bool isValid();
    bool setNextIndex();
    int getApproachesNumber();

    SoSeparator* getSep();

    VirtualRobot::RobotNodePtr getTCP();

    bool setEEFPose(const Eigen::Matrix4f &pose);
    bool updateEEFPose(const Eigen::Vector3f& deltaPosition);
    bool updateEEFPose(const Eigen::Matrix4f& deltaPose);
    Eigen::Matrix4f getEEFPose();

    bool areMoreSegments();
    SoSeparator* getPlanes();

protected:

    SimoxCGAL::SkeletonPtr skeleton;
    SimoxCGAL::SurfaceMeshPtr mesh;
    SimoxCGAL::SegmentedObjectPtr segmentation;
    DeciderGraspPreshapePtr decider;

    PrincipalAxis3D pca;
    VirtualRobot::MathTools::Plane plane;
    std::vector<Eigen::Vector3f> approachDirs;

    int currentSubpart;
    int currentSkeletonVertex;
    bool calculatedApproaches;

    bool noMoreGrasps;
    bool validGrasp;

    void calculateApproaches();
    void calculateApproachDirRound(PrincipalAxis3D &pca);
    void calculateApproachDirRectangular(PrincipalAxis3D &pca);
    int samplingSkeleton(float dist);


    //test
    SoSeparator* sep;
    Eigen::Vector3f testV;

private:
    bool init();

};

typedef boost::shared_ptr<ApproachMovementSkeleton> ApproachMovementSkeletonPtr;

#endif /* __APPROACH_MOVEMENT_SKELETON_H__ */
