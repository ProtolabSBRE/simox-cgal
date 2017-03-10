#ifndef SKELETONVISUALIZATION_H
#define SKELETONVISUALIZATION_H

#include <Eigen/Dense>
#include <stdlib.h>

#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>


#include "../Segmentation/Skeleton/SkeletonPart.h"

#include "SimoxCGAL.h"

class SkeletonVisualization
{
public:

    static SoSeparator* createSegmentationVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, std::vector<SimoxCGAL::ObjectPartPtr> members, bool show_lines);
    static SoSeparator* createSegmentVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, SimoxCGAL::SkeletonPartPtr subpart, bool show_lines);

    static SoSeparator* createPigmentedMeshVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, std::vector<SimoxCGAL::ObjectPartPtr> members, int part);
    static SoNode* createPigmentedSubpartVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, SimoxCGAL::SkeletonPartPtr subpart, VirtualRobot::VisualizationFactory::Color color);

    static SoSeparator* createSkeletonVisualization(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, bool showLines);

    static SoIndexedLineSet* createConnectionVisualization(SimoxCGAL::SkeletonVertex &vertex, SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh);
    static SoIndexedLineSet* createPolylinesVisualization(Eigen::Vector3f center, std::vector<Eigen::Vector3f> lines);


protected:
    SkeletonVisualization();
    ~SkeletonVisualization();
};

#endif // SKELETONVISUALIZATION_H
