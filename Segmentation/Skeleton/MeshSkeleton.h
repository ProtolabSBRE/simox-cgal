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

#ifndef _SimoxCGAL_MeshSkeleton_h_
#define _SimoxCGAL_MeshSkeleton_h_

#include "SimoxCGAL.h"
#include "CGALSurfaceMesh.h"
#include "SkeletonPoint.h"
#include "../SegmentedObject.h"
#include "SkeletonPart.h"

namespace SimoxCGAL
{

    /*!
     * \brief The MeshSkeleton class holds information about the surface mesh and its skeleton structrue. In addition, the segmented object structure can be accessed
     */
    class SIMOX_CGAL_IMPORT_EXPORT MeshSkeleton
    {
    public:

        MeshSkeleton(CGALSurfaceMeshPtr mesh, SkeletonPtr skeleton, double width = 0.0);

        MeshSkeleton(CGALSurfaceMeshPtr mesh, SkeletonPtr skeleton, SegmentedObjectPtr segmentedObject);

        /*!
        */
        virtual ~MeshSkeleton();

        SimoxCGAL::SegmentedObjectPtr getSegmentedObject();

        int getTime();

        void print();

    protected:

        bool segmentationSkeleton();
        bool searchBranches();
        void recursionBranchSegmentation(SkeletonPtr skeleton, SkeletonVertex center, float width);
        void recursionEndpointSegmentation(SkeletonVertex center, SkeletonPartPtr subpart);
        void updateVertices();


        int segmentationTimeMS;


        std::vector<int> id_map;
        std::vector<SimoxCGAL::SkeletonVertex> branchList;
        std::list<SimoxCGAL::SkeletonVertex> endpointList;
        std::vector<SkeletonPointPtr> pointMap;


        CGALSurfaceMeshPtr mesh;
        SkeletonPtr skeleton;
        float width;

        SimoxCGAL::SegmentedObjectPtr segmentedObject;

    };

    typedef boost::shared_ptr<MeshSkeleton> MeshSkeletonPtr;
}

#endif

