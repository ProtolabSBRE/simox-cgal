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

#ifndef _SimoxCGAL_SimoxCGAL_H
#define _SimoxCGAL_SimoxCGAL_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/properties_Surface_mesh.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>

#include <VirtualRobot/VirtualRobot.h>
#include "SimoxCGALImportExport.h"

namespace SimoxCGAL
{

typedef CGAL::Simple_cartesian<double>									Kernel;

typedef Kernel::Point_3                                                 Point;
typedef Kernel::Vector_3                                                Vector;

// CGAL mesh defined as surface_mesh
typedef CGAL::Surface_mesh<Point>										SurfaceMesh;
typedef boost::shared_ptr<SurfaceMesh>                                  SurfaceMeshPtr;

typedef boost::graph_traits<SurfaceMesh>::vertex_descriptor             SurfaceMeshVertexDescriptor;
typedef boost::graph_traits<SurfaceMesh>::halfedge_descriptor           SurfaceMeshHalfedgeDescriptor;
typedef boost::graph_traits<SurfaceMesh>::face_descriptor               SurfaceMeshFaceDescriptor;
//typedef boost::graph_traits<SurfaceMesh>::face_descriptor               SurfaceMeshFaceDesriptor;

typedef CGAL::Mean_curvature_flow_skeletonization<SurfaceMesh>          Skeletonization;
typedef Skeletonization::Skeleton                          				Skeleton;
typedef boost::shared_ptr<Skeleton>                                     SkeletonPtr;
typedef Skeleton::vertex_descriptor                             		SkeletonVertex;
typedef boost::graph_traits<Skeleton>::adjacency_iterator      			SkeletonAdjacency;

typedef Skeleton::edge_descriptor                             			SkeletonEdge;


// CGAL mesh defined as polyhedron_3 mesh
typedef CGAL::Exact_predicates_inexact_constructions_kernel             KernelPolyhedron;
typedef CGAL::Polyhedron_3<KernelPolyhedron>                            PolyhedronMesh;
typedef boost::shared_ptr<PolyhedronMesh>                               PolyhedronMeshPtr;
//typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>	Polyhedron;
//typedef boost::shared_ptr<Polyhedron>                                   PolyhedronPtr;
typedef KernelPolyhedron::Point_3                                       PointPoly;
typedef CGAL::Point_with_normal_3<KernelPolyhedron>                     PointNormalPoly;

typedef PolyhedronMesh::Vertex                                          PolyVertex;
typedef PolyhedronMesh::Vertex_const_handle                             PolyVertexConstHandle;
typedef PolyhedronMesh::Vertex_iterator                             	PolyVertexIterator;
typedef PolyhedronMesh::Facet_iterator                              	PolyFacetIterator;
typedef PolyhedronMesh::Halfedge_around_facet_circulator                PolyHalfedgeFacetCirculator;
typedef PolyhedronMesh::Halfedge_handle                             	PolyHalfedgeHandle;
typedef boost::graph_traits<PolyhedronMesh>::face_descriptor            PolyFaceDescriptor;
typedef boost::graph_traits<PolyhedronMesh>::halfedge_descriptor        PolyHalfedgeDescriptor;

typedef std::map<PolyhedronMesh::Facet_const_handle, double>            PolyhedronFacetDoubleMap;
typedef std::map<PolyhedronMesh::Facet_const_handle, int>               PolyhedronFacetIntMap;
typedef std::map<int, int>                                              SkeletonIdMap;

    /*!
        Initialize the runtime envionment and add data paths.
    */
    void SIMOX_CGAL_IMPORT_EXPORT init(int &argc, char* argv[], const std::string &appName);
    void SIMOX_CGAL_IMPORT_EXPORT init(const std::string &appName);

}

#endif // _SimoxCGAL_SimoxCGAL_H
