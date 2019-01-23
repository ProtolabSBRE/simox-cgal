#include "MeshReconstruction.h"

#include <VirtualRobot/Visualization/TriMeshModel.h>

#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/compute_average_spacing.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Scale_space_surface_reconstruction_3.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>

#include <vector>
#include <fstream>
#include <utility>
#include <functional>

using namespace VirtualRobot;
using namespace std;

namespace SimoxCGAL {

MeshReconstruction::MeshReconstruction(): verbose(false)
{
}


MeshReconstruction::~MeshReconstruction()
{

}

bool MeshReconstruction::regularizePoints(std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &normals, float cellSize)
{
    typedef KernelPolyhedron::Point_3 Point;
    typedef KernelPolyhedron::Vector_3 Vector;

    if (points.size()==0)
        return false;

    if (points.size()!=normals.size() && normals.size()>0)
        return false;

    std::vector<Point> pointsC;
    std::vector<Vector> normalsC;
    if (verbose)
      VR_INFO << points.size() << " input points" << std::endl;

    std::vector<std::size_t> indices(points.size());
    for(std::size_t i = 0; i < points.size(); ++i)
    {
        pointsC.push_back(Point(points.at(i)[0], points.at(i)[1], points.at(i)[2]));
        if (normals.size()>0)
            normalsC.push_back(Vector(normals.at(i)[0], normals.at(i)[1], normals.at(i)[2]));
        indices[i] = i;
    }
    // simplification by clustering using erase-remove idiom
    std::vector<std::size_t>::iterator end;
    end = CGAL::grid_simplify_point_set(indices.begin(),
                                      indices.end(),
                                      CGAL::make_property_map(pointsC),
                                      cellSize);
    std::size_t k = end - indices.begin();
    if (verbose)
    VR_INFO << "Keep " << k << " of " << indices.size() <<  " indices" << std::endl;


    std::vector<Eigen::Vector3f> tmp_points(k);
    std::vector<Eigen::Vector3f> tmp_normals(k);
    for(std::size_t i=0; i<k; ++i)
    {
      tmp_points[i] = points[indices[i]];
      if (normals.size()>0)
        tmp_normals[i] = normals[indices[i]];
    }
    points.swap(tmp_points);
    normals.swap(tmp_normals);
    return true;
}

bool MeshReconstruction::computeNormals(std::vector<Eigen::Vector3f> &points, std::vector<Eigen::Vector3f> &storeNormals, bool erasePointsWithWrongNormal)
{

    // Point with normal vector stored in a std::pair.
    typedef std::pair<Point, Vector> PointVectorPair;

    // Concurrency
    #ifdef CGAL_LINKED_WITH_TBB
    typedef CGAL::Parallel_tag Concurrency_tag;
    #else
    typedef CGAL::Sequential_tag Concurrency_tag;
    #endif
    std::list<PointVectorPair> cgPoints;
    for (size_t i=0; i<points.size(); i++)
    {
        const Eigen::Vector3f &p = points.at(i);
        cgPoints.push_back(std::make_pair(Point(p[0], p[1], p[2]), Vector(0,0,0)));
    }

    // Estimates normals direction.
    // Note: pca_estimate_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
    CGAL::pca_estimate_normals<Concurrency_tag>(cgPoints.begin(), cgPoints.end(),
                               CGAL::First_of_pair_property_map<PointVectorPair>(),
                               CGAL::Second_of_pair_property_map<PointVectorPair>(),
                               nb_neighbors);
    // Orients normals.
    // Note: mst_orient_normals() requires an iterator over points
    // as well as property maps to access each point's position and normal.
    std::list<PointVectorPair>::iterator unoriented_points_begin =
      CGAL::mst_orient_normals(cgPoints.begin(), cgPoints.end(),
                                 CGAL::First_of_pair_property_map<PointVectorPair>(),
                                 CGAL::Second_of_pair_property_map<PointVectorPair>(),
                                 nb_neighbors);
    // Optional: delete points with an unoriented normal
    // if you plan to call a reconstruction algorithm that expects oriented normals.
    if (erasePointsWithWrongNormal)
        cgPoints.erase(unoriented_points_begin, cgPoints.end());

    size_t origPointSize = points.size();

    storeNormals.clear();
    if (erasePointsWithWrongNormal)
        points.clear();
    for (PointVectorPair &p : cgPoints)
    {
        if (erasePointsWithWrongNormal)
            points.push_back(Eigen::Vector3f(p.first[0], p.first[1], p.first[2]));
        storeNormals.push_back(Eigen::Vector3f(p.second[0], p.second[1], p.second[2]));
    }

    VR_INFO << "Oriented " << storeNormals.size() << " normals. Could not coherently orient " << origPointSize-storeNormals.size() << " normals." << endl;

    VR_ASSERT(points.size() == storeNormals.size());
    return true;
}

VirtualRobot::TriMeshModelPtr MeshReconstruction::reconstructMeshScaleSpace(std::vector<Eigen::Vector3f> &points)
{
    if (points.size()==0)
        return TriMeshModelPtr();
    if (verbose)
        VR_INFO << "Converting " << points.size() << " points to cgal data structure" << endl;

    typedef CGAL::Scale_space_surface_reconstruction_3< KernelPolyhedron >    Reconstruction;
    typedef CGAL::Scale_space_reconstruction_3::Weighted_PCA_smoother< KernelPolyhedron > Smoother;
    typedef CGAL::Scale_space_reconstruction_3::Alpha_shape_mesher< KernelPolyhedron >    Mesher;
    typedef Reconstruction::Point                                   Point;
    typedef CGAL::Timer Timer;

    // construct the data.
    std::vector< Point > pointsC;
    for (Eigen::Vector3f &p : points)
    {
        pointsC.push_back(Point(p[0],p[1],p[2]));
    }



    Timer t;
    t.start();
    // Construct the mesh in a scale space.
    Reconstruction reconstruct;
    reconstruct.insert( pointsC.begin(), pointsC.end() );
    //reconstruct.increase_scale( 2 );
    Smoother smoother( 20, 800 );
    Mesher mesher (smoother.squared_radius(), true, false, 4);
    reconstruct.reconstruct_surface(mesher);
    //reconstruct.increase_scale( 4 );
    //reconstruct.reconstruct_surface(8, true, false);//, false, true );
    if (verbose)
    {
        VR_INFO << "Reconstruction done in " << t.time() << " sec." << std::endl;
        VR_INFO << "Number of shells: " << mesher.number_of_shells() << std::endl;
        VR_INFO << "Neighborhood radius^2: " << smoother.squared_radius() << std::endl;
    }
    t.reset();

    // Convert the reconstruction.
    TriMeshModelPtr tm(new TriMeshModel());

    for( std::size_t shell = 0; shell < mesher.number_of_shells(); ++shell ) {
        for( auto it = mesher.shell_begin( shell ); it != mesher.shell_end( shell ); ++it )
        {
            if (it->at(0)>points.size())
            {
                VR_ERROR << "Index 0 " << it->at(0) << " out of bounds" << endl;
                continue;
            }
            if (it->at(1)>points.size())
            {
                VR_ERROR << "Index 1 " << it->at(1) << " out of bounds" << endl;
                continue;
            }
            if (it->at(2)>points.size())
            {
                VR_ERROR << "Index 0 " << it->at(2) << " out of bounds" << endl;
                continue;
            }
            tm->addTriangleWithFace(points.at(it->at(0)), points.at(it->at(1)), points.at(it->at(2)));
        }
          //out << "3 "<< *it << '\n'; // We write a '3' in front so that it can be assembled into an OFF file
    }
    if (verbose)
    {
        VR_INFO << "Converted result in " << t.time() << " sec." << std::endl;
    }
    return tm;
}

void MeshReconstruction::setVerbose(bool v)
{
    verbose = v;
}

PolyhedronMeshPtr MeshReconstruction::reconstructMeshPoisson(const std::vector<Eigen::Vector3f> &points, const std::vector<Eigen::Vector3f> &normals, bool parameterFillHoles)
{
    if (points.size() != normals.size())
    {
        VR_ERROR << "Size of points != size of normals" << endl;
        return PolyhedronMeshPtr();
    }
    if (points.size()==0)
        return PolyhedronMeshPtr();
    if (verbose)
        VR_INFO << "Converting " << points.size() << " points with normals to cgal data structure" << endl;

    std::vector<PointNormalPoly> pointsNormals;
    pointsNormals.reserve(points.size());
    for (size_t i=0; i<points.size(); i++)
    {
        const Eigen::Vector3f &p = points.at(i);
        const Eigen::Vector3f &n = normals.at(i);
        PointNormalPoly::Vector v(n[0], n[1], n[2]);
        PointNormalPoly pn(p[0], p[1], p[2], v);
        pointsNormals.push_back(pn);
    }

    return reconstructMeshPoisson(pointsNormals, parameterFillHoles);
}


PolyhedronMeshPtr MeshReconstruction::reconstructMeshPoisson(std::vector<PointNormalPoly> &points, bool parameterFillHoles)
{
    if (points.size()==0)
    {
        VR_ERROR << "no points to reconstruct..." << endl;
        return PolyhedronMeshPtr();
    }

    if (verbose)
        VR_INFO << "Starting to reconstruct mesh" << endl;

    // Poisson options
    KernelPolyhedron::FT sm_angle = 20.0; // Min triangle angle in degrees.
    KernelPolyhedron::FT sm_radius = 30; // Max triangle size w.r.t. point set average spacing.
    KernelPolyhedron::FT sm_distance = 0.375; // Surface Approximation error w.r.t. point set average spacing.


    if (verbose)
        VR_INFO << "Creating implicit function.." << endl;

    // Creates implicit function from the read points using the default solver.
    // Note: this method requires an iterator over points
    // + property maps to access each point's position and normal.
    // The position property map can be omitted here as we use iterators over Point_3 elements.
    CGAL::Poisson_reconstruction_function<KernelPolyhedron> function(points.begin(), points.end(),
                                             CGAL::make_normal_of_point_with_normal_map(std::vector<PointNormalPoly>::value_type()) );
    // Computes the Poisson indicator function f()
    // at each vertex of the triangulation.
    if ( ! function.compute_implicit_function(parameterFillHoles) )
    {
        VR_ERROR << "Could not compute implicit function..." << endl;
        return PolyhedronMeshPtr();
    }
    if (verbose)
        VR_INFO << "Compute average spacing.." << endl;
    // Computes average spacing
    KernelPolyhedron::FT average_spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(points.begin(), points.end(),
                                                       6 /* knn = 1 ring */);
    // Gets one point inside the implicit surface
    // and computes implicit function bounding sphere radius.
    PointNormalPoly inner_point = function.get_inner_point();
    KernelPolyhedron::Sphere_3 bsphere = function.bounding_sphere();
    KernelPolyhedron::FT radius = std::sqrt(bsphere.squared_radius());

    if (verbose)
        VR_INFO << "Computing surface.." << endl;

    // Defines the implicit surface: requires defining a
    // conservative bounding sphere centered at inner point.
    KernelPolyhedron::FT sm_sphere_radius = 5.0 * radius;
    KernelPolyhedron::FT sm_dichotomy_error = sm_distance*average_spacing/1000.0; // Dichotomy error must be << sm_distance
    CGAL::Implicit_surface_3< KernelPolyhedron, CGAL::Poisson_reconstruction_function<KernelPolyhedron> > surface(function,
                      KernelPolyhedron::Sphere_3(inner_point,sm_sphere_radius*sm_sphere_radius),
                      sm_dichotomy_error/sm_sphere_radius);
    // Defines surface mesh generation criteria
    CGAL::Surface_mesh_default_criteria_3<CGAL::Surface_mesh_default_triangulation_3> criteria(sm_angle,  // Min triangle angle (degrees)
                                                        sm_radius*average_spacing,  // Max triangle size
                                                        sm_distance*average_spacing); // Approximation error
    if (verbose)
        VR_INFO << "Generating mesh.." << endl;
    // Generates surface mesh with manifold option
    CGAL::Surface_mesh_default_triangulation_3 tr; // 3D Delaunay triangulation for surface mesh generation
    CGAL::Surface_mesh_complex_2_in_triangulation_3<CGAL::Surface_mesh_default_triangulation_3> c2t3(tr); // 2D complex in 3D Delaunay triangulation
    CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_with_boundary_tag());  // require manifold mesh
    if(tr.number_of_vertices() == 0)
    {
        VR_ERROR << "Could not compute surface..." << endl;
        return PolyhedronMeshPtr();
    }

    if (verbose)
        VR_INFO << "Converting mesh to polyhedron..." << endl;
    // saves reconstructed surface mesh
    //std::ofstream out("kitten_poisson-20-30-0.375.off");
    PolyhedronMeshPtr result(new PolyhedronMesh());
    CGAL::output_surface_facets_to_polyhedron(c2t3, *result);
    //out << output_mesh;
    return result;
}

}
