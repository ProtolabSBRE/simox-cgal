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

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>


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

    protected:
        static SoSeparator* CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VirtualRobot::VisualizationFactory::PhongMaterial mat, VirtualRobot::VisualizationFactory::Color colorLine, float lineSize);

        CGALCoinVisualization() {}
        ~CGALCoinVisualization() {}
    };
}

#endif // _SimoxCGAL_CGALCoinVisualization_h_
