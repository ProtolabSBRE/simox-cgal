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

#ifndef _SimoxCGAL_CGALMeshConverter_h_
#define _SimoxCGAL_CGALMeshConverter_h_

#include "SimoxCGAL.h"
#include "CGALSurfaceMesh.h"
#include "CGALPolyhedronMesh.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>

namespace SimoxCGAL
{


    class CGALPolyhedronMeshBuilder : public CGAL::Modifier_base<PolyhedronMesh::HalfedgeDS>
    {
    public:
        CGALPolyhedronMeshBuilder(VirtualRobot::TriMeshModelPtr &tm);
        void operator()(PolyhedronMesh::HalfedgeDS& hds);

    private:
        VirtualRobot::TriMeshModelPtr tm;

    };


    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT CGALMeshConverter
    {
    public:

        static CGALSurfaceMeshPtr ConvertToSurfaceMesh(VirtualRobot::TriMeshModelPtr tm, bool trimeshAlreadyCGALCompatible = false);
        static CGALPolyhedronMeshPtr ConvertToPolyhedronMesh(VirtualRobot::TriMeshModelPtr tm, bool trimeshAlreadyCGALCompatible = false);
        static VirtualRobot::TriMeshModelPtr ConvertCGALMesh(CGALSurfaceMeshPtr m);
        static VirtualRobot::TriMeshModelPtr ConvertCGALMesh(CGALPolyhedronMeshPtr m);

        /*!
         * \brief ConvertTrimeshCGALCompatible Make internal trimesh structure compatible to cgal
         * \param tm
         * \return The novel trimesh which can be converted to cgal
         */
        static VirtualRobot::TriMeshModelPtr ConvertTrimeshCGALCompatible(VirtualRobot::TriMeshModelPtr tm);

        //static CGALPolyhedronMeshPtr PolygonSoupToPolyhedronMesh(VirtualRobot::TriMeshModelPtr tm);

    private:
        // no need to instanciate this class
        CGALMeshConverter();
        virtual ~CGALMeshConverter();
    };

}

#endif // _SimoxCGAL_CGALMeshConverter_h_
