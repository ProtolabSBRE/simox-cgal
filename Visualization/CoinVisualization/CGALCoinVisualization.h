/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    SimoxCGAL
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/


#ifndef _SimoxCGAL_CGALCoinVisualization_h_
#define _SimoxCGAL_CGALCoinVisualization_h_

#include "SimoxCGAL.h"
#include "CGALSurfaceMesh.h"
#include "CGALPolyhedronMesh.h"
#include "ObjectPart.h"
#include "CGALSkeleton.h"
#include "Segmentation/Skeleton/SkeletonPart.h"
#include "GraspPlanning/Skeleton/SkeletonVertexAnalyzer.h"

#include <Inventor/nodes/SoIndexedLineSet.h>

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/ManipulationObject.h>


class SoNode;
class SoSeparator;

namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT CGALCoinVisualization
    {
    public:

        static SoNode* CreateCoinVisualization(CGALPolyhedronMeshPtr mesh, bool showLines = false, bool showNormals = false);
        static SoNode* CreateCoinVisualizationSDF(CGALPolyhedronMeshPtr mesh,  boost::associative_property_map<PolyhedronFacetDoubleMap>& sdf_raw_property_map, float maxSDF, bool showLines = false, bool showNormals = false);
        static SoNode* CreateCoinVisualizationSegments(CGALPolyhedronMeshPtr mesh,  boost::associative_property_map<PolyhedronFacetIntMap>& segment_property_map, size_t number_of_segments, bool showLines = false, bool showNormals = false);
        static SoNode* CreateCoinVisualization(CGALPolyhedronMeshPtr mesh,
                                                boost::associative_property_map<PolyhedronFacetDoubleMap>& sdf_raw_property_map, // if entries, -> create sdf visu
                                               float maxSDF,
                                                boost::associative_property_map<PolyhedronFacetIntMap>& segment_property_map, // if entries -> create segment visu
                                               size_t number_of_segments,
                                               bool showLines,
                                               bool showNormals);



        static SoSeparator* CreateSegmentationVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, std::vector<ObjectPartPtr> members,
                                                            bool show_lines,
                                                            float pointSize = 1.0f,
                                                            float lineWidth = 10.0f);
        static SoSeparator* CreateSegmentVisualization(SkeletonPtr skeleton,
                                                       SurfaceMeshPtr mesh,
                                                       SkeletonPartPtr subpart,
                                                       bool show_lines,
                                                       float pointSize = 1.0f,
                                                       float lineWidth = 10.0f);

        static SoSeparator* CreatePigmentedMeshVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, std::vector<ObjectPartPtr> members, int part);
        static SoNode* CreatePigmentedSubpartVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SkeletonPartPtr subpart);

        static SoSeparator* CreateSkeletonVisualization(SkeletonPtr skeleton,
                                                        SurfaceMeshPtr mesh,
                                                        bool showLines,
                                                        float pointSize = 1.0f,
                                                        float lineWidth = 10.0f);
        static SoSeparator* ShowSkeletonPoint(SkeletonPtr skeleton, SurfaceMeshPtr mesh, int pointPosition);

        static SoSeparator* CreateConnectionVisualization(SkeletonVertex &vertex, SkeletonPtr skeleton, SurfaceMeshPtr mesh,
                                                          float lineWidth = 10.0f);
        static SoSeparator* CreatePolylinesVisualization(Eigen::Vector3f center, std::vector<Eigen::Vector3f> lines, float lineWidth = 1.0f);

        static SoSeparator* CreateGraspVisualization(VirtualRobot::GraspPtr grasp, VirtualRobot::ManipulationObjectPtr object);

        static SoSeparator* CreateGraspOnSurfaceVisualization(VirtualRobot::GraspPtr grasp,
                                                              VirtualRobot::EndEffectorPtr eef,
                                                              VirtualRobot::ManipulationObjectPtr object,
                                                              CGALPolyhedronMeshPtr polyhedronObject,
                                                              float sizePoint = 1.0f, float lineSize = 1.0f, float lineLength = 50.0f);
        static SoSeparator* CreateGraspsOnSurfaceVisualization(VirtualRobot::GraspSetPtr grasps,
                                                              VirtualRobot::EndEffectorPtr eef,
                                                              VirtualRobot::ManipulationObjectPtr object,
                                                              CGALPolyhedronMeshPtr polyhedronObject,
                                                              float sizePoint = 1.0f, float lineSize = 1.0f, float lineLength = 50.0f);


        static SoSeparator* CreateGraspOnSurfaceVisualization(VirtualRobot::GraspPtr grasp,
                                                              VirtualRobot::EndEffectorPtr eef,
                                                              VirtualRobot::ManipulationObjectPtr object,
                                                              PolyTree &polyhedronTree,
                                                              float sizePoint = 1.0f, float lineSize = 1.0f, float lineLength = 50.0f);



        static SoNode* CreateGraspIntervalVisualization(const SkeletonVertexResult &result,
                                                        SurfaceMeshPtr mesh,
                                                        bool showPoints = true,
                                                        VirtualRobot::VisualizationFactory::Color colorPoints = VirtualRobot::VisualizationFactory::Color::Red(),
                                                        VirtualRobot::VisualizationFactory::Color colorPlane1 = VirtualRobot::VisualizationFactory::Color::Red(),
                                                        VirtualRobot::VisualizationFactory::Color colorPlane2 = VirtualRobot::VisualizationFactory::Color::Green());

        static SoNode* CreateProjectedPointsVisualization(const SimoxCGAL::SkeletonVertexResult result,
                                                          SurfaceMeshPtr mesh,
                                                          VirtualRobot::VisualizationFactory::Color colorPoints = VirtualRobot::VisualizationFactory::Color::Red(),
                                                          VirtualRobot::VisualizationFactory::Color colorPlane = VirtualRobot::VisualizationFactory::Color::Green());


    protected:

        static SoSeparator* verticesVisu(std::vector<Eigen::Vector3f> &v, float pointSize);
        static SoSeparator* CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VirtualRobot::VisualizationFactory::PhongMaterial mat, VirtualRobot::VisualizationFactory::Color colorLine, float lineSize);


        CGALCoinVisualization() {}
        ~CGALCoinVisualization() {}
    };
}

#endif // _SimoxCGAL_CGALCoinVisualization_h_
