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

// now defined via cmake
//#define CGAL_EIGEN3_ENABLED

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/boost/graph/properties_Surface_mesh.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polyhedron_items_with_id_3.h>

#include <VirtualRobot/VirtualRobot.h>
#include "SimoxCGALImportExport.h"

namespace SimoxCGAL
{

typedef CGAL::Simple_cartesian<double>									Kernel;

typedef Kernel::Point_3													Point;

typedef CGAL::Surface_mesh<Point>										TriangleMesh;
typedef boost::shared_ptr<TriangleMesh>                                 TriangleMeshPtr;

typedef boost::graph_traits<TriangleMesh>::vertex_descriptor			MeshVertexDescriptor;
typedef boost::graph_traits<TriangleMesh>::face_descriptor				MeshFaceDesriptor;

typedef CGAL::Mean_curvature_flow_skeletonization<TriangleMesh>         Skeletonization;
typedef Skeletonization::Skeleton                          				Skeleton;
typedef boost::shared_ptr<Skeleton>                                     SkeletonPtr;
typedef Skeleton::vertex_descriptor                             		SkeletonVertex;
typedef boost::graph_traits<Skeleton>::adjacency_iterator      			SkeletonAdjacency;

typedef Skeleton::edge_descriptor                             			SkeletonEdge;


typedef CGAL::Polyhedron_3<Kernel, CGAL::Polyhedron_items_with_id_3>	Polyhedron;
typedef boost::shared_ptr<Polyhedron>                                   PolyhedronPtr;
typedef Polyhedron::Vertex_iterator                             		PolyVertexIterator;
typedef Polyhedron::Facet_iterator                              		PolyFacetIterator;
typedef Polyhedron::Halfedge_handle                             		HalfedgeHandle;
typedef boost::graph_traits<Polyhedron>::face_descriptor        		FaceDescriptor;
typedef boost::graph_traits<Polyhedron>::halfedge_descriptor    		HalfedgeDescriptor;

}

#endif // _SimoxCGAL_SimoxCGAL_H
