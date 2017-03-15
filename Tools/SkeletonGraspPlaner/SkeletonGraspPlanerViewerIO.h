#ifndef SKELETONVIEWERWINDOWIO_H
#define SKELETONVIEWERWINDOWIO_H

#include <string>
#include "SimoxCGAL.h"
#include "SegmentedObject.h"
#include "CGALSurfaceMesh.h"
#include "CGALSkeleton.h"
#include "Segmentation/Skeleton/MeshSkeleton.h"

struct LoadedData
{
    VirtualRobot::ManipulationObjectPtr manipObject;
    SimoxCGAL::MeshSkeletonPtr segSkeleton;
    SimoxCGAL::CGALSurfaceMeshPtr surfaceMesh;
    SimoxCGAL::CGALSkeletonPtr skeleton;
};

class SkeletonGraspPlanerViewerIO
{

public:

    static bool saveSkeletonViewerData(const std::string& basePath, const std::string& objectFile, SimoxCGAL::CGALSkeletonPtr skeleton, SimoxCGAL::CGALSurfaceMeshPtr mesh, SimoxCGAL::SegmentedObjectPtr segmentedObject);
    static LoadedData loadSkeletonViewerData(const std::string& filename);
//    static bool saveGrasps();
//    static VirtualRobot::GraspSetPtr loadGrasps();


protected:

    //no instace needed
    SkeletonGraspPlanerViewerIO();
    ~SkeletonGraspPlanerViewerIO();
};

#endif // SKELETONVIEWERWINDOWIO_H
