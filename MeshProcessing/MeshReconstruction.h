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

#ifndef _SimoxCGAL_MeshReconstruction_h_
#define _SimoxCGAL_MeshReconstruction_h_

#include "SimoxCGAL.h"
#include "CGALSurfaceMesh.h"

namespace SimoxCGAL
{
    /*!

    */
    class SIMOX_CGAL_IMPORT_EXPORT MeshReconstruction
    {
    public:

        MeshReconstruction();

        /*!
        */
        virtual ~MeshReconstruction();


        /*!
         * \brief reconstructMeshPoisson Use the poisson algorithm to reconstruct the mesh. Needs points and normals.
         * @see http://doc.cgal.org/latest/Poisson_surface_reconstruction_3
         * \param points The points with normals
         * \return The mesh
         */
        PolyhedronMeshPtr reconstructMeshPoisson(std::vector<PointNormalPoly> &points, bool parameterFillHoles = true);
        PolyhedronMeshPtr reconstructMeshPoisson(const std::vector<Eigen::Vector3f> &points, const std::vector<Eigen::Vector3f> &normals, bool parameterFillHoles = true);

        /*!
         * \brief reconstructMeshScaleSpace Use the scale space algorithm to reconstruct the mesh. Works on unordered point sets.
         * @see http://doc.cgal.org/latest/Scale_space_reconstruction_3/
         * \param points The points
         * \return Teh mesh
         */
        VirtualRobot::TriMeshModelPtr reconstructMeshScaleSpace(std::vector<Eigen::Vector3f> &points);

        /*!
         * \brief regularizePoints use the grid algorithm to reguralize the points
         * @see doc.cgal.org/latest/Point_set_processing_3
         * \param points content will be changed
         * \param normals may be empty, otherwise the vector needs to have the same size as the points vector.
         * \return
         */
        bool regularizePoints(std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &normals, float cellSize = 1.0f);

        void setVerbose(bool v);

    protected:
        bool verbose;

        CGALSurfaceMeshPtr mesh;
    };

    typedef boost::shared_ptr<MeshReconstruction> MeshReconstructionPtr;
}

#endif

