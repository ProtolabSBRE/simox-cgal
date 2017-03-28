#ifndef __APPROACH_MOVEMENT_SKELETON_H__
#define __APPROACH_MOVEMENT_SKELETON_H__

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/ApproachMovementGenerator.h>
#include <VirtualRobot/SceneObject.h>
#include <vector>
#include "VirtualRobot/Nodes/RobotNode.h"

#include "SimoxCGAL.h"
#include "SegmentedObject.h"
#include "SkeletonVertexAnalyzer.h"
#include "DeciderGraspPreshape.h"
#include "Segmentation/Skeleton/SkeletonPoint.h"
/*
#define POWER_GRASP "Power Preshape"
#define POWER_INTERVALL 94.f

#define PRECISION_GRASP "Precision Preshape"
#define PRECISION_INTERVALL 47.f

#define RATIO_THREASHOLD 1.3f

#define SAMPLING_LENGTH 0.f
*/

namespace SimoxCGAL
{
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
    struct PlanningParameters
    {
        enum GraspType
        {
            Power,
            Precision
        };

        PlanningParameters()
        {
            // ARMAR-III parameters
            interval[Power] = 94.0f;
            interval[Precision] = 47.0f;
            preshapeName[Power] = "Power Preshape";
            preshapeName[Precision] = "Precision Preshape";

            // pca ratio of first two eigenvalues to identify round vs rectangular objects
            roundThreshold = 1.3f;

            // 0 -> chose next enughbor when going along the skeleton
            skeletonSamplingLength = 0.0f;
        }

        std::map<GraspType, float> interval;
        std::map<GraspType, std::string> preshapeName;

        float roundThreshold;
        float skeletonSamplingLength;

    };

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
    bool calculateApproachDirection();

    //! Sets EEF to a position so that the Z component of the GCP coord system is aligned with -approachDir
    bool setEEFToApproachPose(const Eigen::Vector3f& position, const Eigen::Vector3f& approachDir, bool invertOrientation, const VirtualRobot::MathTools::Plane &plane);

    //! Move endeffector away until valid position.
    bool moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops = 1000);

    //! Set EndEffector to predefined position at prehsape
    bool setEEFPose(const Eigen::Matrix4f &pose);

    bool updateEEFPose(const Eigen::Matrix4f& deltaPose);
    bool updateEEFPose(const Eigen::Vector3f& deltaPosition);


    SoSeparator* getVisualization();



    std::string getGraspPreshape();

    /*!
     * \brief next Go to next vertex on skeleton, switch segment if needed
     */
    bool nextSkeletonVertex();

    bool isValid();
    bool setNextIndex();
    int getApproachesNumber();

    // returns true, if all segments and vertices have been processed
    bool finished();

    int getCurrentSegment();
    int getCurrentVertex();

    int getSegmentCount();
    int getVertexCount(int segNr);


    VirtualRobot::RobotNodePtr getTCP();

    Eigen::Matrix4f getEEFPose();

    bool moreSegmentsAvailable();
    SoSeparator* getPlanes();

    PlanningParameters getParameters();
    void setParameters(PlanningParameters &p);

protected:

    std::vector<Eigen::Vector3f> approachDirs;

    SimoxCGAL::SkeletonPtr skeleton;
    SimoxCGAL::SurfaceMeshPtr mesh;
    SimoxCGAL::SegmentedObjectPtr segmentation;
    DeciderGraspPreshapePtr decider;

    //PrincipalAxis3D pca;
    //VirtualRobot::MathTools::Plane plane;

    int currentSubpart;
    int currentSkeletonVertex;
    bool approachDirectionsCalculated;

    void calculateApproachesConnectionPoint(const PrincipalAxis3D &pca);
    void calculateApproachDirRound(const PrincipalAxis3D &pca, bool endpoint = false);
    void calculateApproachDirRectangular(const PrincipalAxis3D &pca, bool endpoint = false);
    void calculateApproachesEndpoint();
    int nextVertexOnCurrentSubpart(float dist);


    //visu
    SoSeparator* visu;

    PlanningParameters approachMovementParameters;
    SkeletonVertexResult currentVertexResult;

private:
    bool init();

};

typedef boost::shared_ptr<ApproachMovementSkeleton> ApproachMovementSkeletonPtr;

}
#endif /* __APPROACH_MOVEMENT_SKELETON_H__ */
