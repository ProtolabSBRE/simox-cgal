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

#ifndef _SimoxCGAL_MeshSDF_h_
#define _SimoxCGAL_MeshSDF_h_

#include "SimoxCGAL.h"
#include "CGALPolyhedronMesh.h"
#include <CGAL/property_map.h>

namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT MeshSDF
    {
    public:

        MeshSDF(CGALPolyhedronMeshPtr mesh,
                size_t paramNrRays = 25,
                double paramConeAngle = 2.0 / 3.0 * M_PI,
                size_t paramNrClusters = 5);

        /*!
        */
        virtual ~MeshSDF();

        float getMinSDF();
        float getMaxSDF();

        boost::associative_property_map<PolyhedronFacetDoubleMap>& getSDFMap();

        boost::associative_property_map<PolyhedronFacetIntMap> &getSegmentMap();

        size_t getNrSegments();

    protected:
        bool checkMeshValid();

        bool segmentationSDF();


        //PolyhedronFacetDoubleMap facetMap;

        //boost::associative_property_map<PolyhedronFacetDoubleMap> sdf_raw_property_map;
        PolyhedronFacetIntMap internal_segment_map;
        boost::associative_property_map<PolyhedronFacetIntMap> segment_property_map;
        size_t number_of_segments;
        int segmentationTimeMS;

        CGALPolyhedronMeshPtr mesh;
        size_t paramNrRays;
        double paramConeAngle;
        size_t paramNrClusters;

        std::pair<double, double> min_max_sdf;
        PolyhedronFacetDoubleMap internal_seg_sdf_map;
        boost::associative_property_map<PolyhedronFacetDoubleMap> sdfMap;

        float minSDF,maxSDF;

    };

    typedef boost::shared_ptr<MeshSDF> MeshSDFPtr;
}

#endif // _SimoxCGAL_MeshSDF_h_
