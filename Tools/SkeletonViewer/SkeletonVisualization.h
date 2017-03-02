#ifndef SKELETONVISUALIZATION_H
#define SKELETONVISUALIZATION_H

#include <Eigen/Dense>
#include <stdlib.h>

#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>


#include "SimoxCGAL.h"

class SkeletonVisualization
{
public:

//    static SoSeparator* createSegmentationVisualization(Skeleton& skeleton, Triangle_mesh& mesh, vector<SubpartPtr>& members, bool show_lines);
//    static SoSeparator* createSegmentVisualization(Skeleton& skeleton, Triangle_mesh& mesh, SubpartPtr& subpart, bool show_lines);
    static SoSeparator* createSkeletonVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, bool showLines);

    static SoIndexedLineSet* createConnectionVisualization(SimoxCGAL::SkeletonVertex &vertex, SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh);
    static SoIndexedLineSet* createPolylinesVisualization(Eigen::Vector3f center, std::vector<Eigen::Vector3f> lines);


protected:
    SkeletonVisualization();
    ~SkeletonVisualization();
};

#endif // SKELETONVISUALIZATION_H
