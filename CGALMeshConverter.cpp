#include "CGALMeshConverter.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>

using namespace VirtualRobot;

namespace SimoxCGAL {


CGALPolyhedronMeshBuilder::CGALPolyhedronMeshBuilder(VirtualRobot::TriMeshModelPtr &tm)
    :tm(tm)
{
}

void CGALPolyhedronMeshBuilder::operator()(PolyhedronMesh::HalfedgeDS &hds)
{
    if (!tm)
        return;
    CGAL::Polyhedron_incremental_builder_3<PolyhedronMesh::HalfedgeDS> B(hds, true);
    //B.ABSOLUTE_INDEXING; macht keinen Unterschied!?! nicht wichtig?
    std::size_t numberVertices =tm->vertices.size();
    std::size_t numberFaces =tm->faces.size();

    typedef typename PolyhedronMesh::HalfedgeDS::Vertex Vertex;
    typedef typename Vertex::Point Point;

    B.begin_surface(numberVertices,numberFaces);
    //add all vertex to cgal
    for(std::vector<Eigen::Vector3f>::iterator itVertices=tm->vertices.begin();itVertices!=tm->vertices.end();itVertices++)
    {
        Eigen::Vector3f tmp=*itVertices;
        Point p1=Point(tmp(0),tmp(1),tmp(2));
        B.add_vertex(p1);
    }

    int nrWrongFacet=0;

    //add face structure
    for(std::vector<VirtualRobot::MathTools::TriangleFace>::iterator itFace=tm->faces.begin();itFace!=tm->faces.end();itFace++)
    {
        // write new InputIterator to test if facet is valid
        std::vector<std::size_t> test_facet;
        test_facet.push_back(itFace->id1);
        test_facet.push_back(itFace->id2);
        test_facet.push_back(itFace->id3);
        if(B.test_facet(test_facet.begin(),test_facet.end()))
        {
            B.begin_facet();
            B.add_vertex_to_facet(itFace->id1);
            B.add_vertex_to_facet(itFace->id2);
            B.add_vertex_to_facet(itFace->id3);
            B.end_facet();
        }
        else
            nrWrongFacet++;
    }
    B.end_surface();
    //std::cout << "Nummer der nicht hinzugefuegten Facet ist: " << nrWrongFacet << std::endl;
}

SimoxCGAL::CGALPolyhedronMeshPtr SimoxCGAL::CGALMeshConverter::ConvertToPolyhedronMesh(VirtualRobot::TriMeshModelPtr tm, bool trimeshAlreadyCGALCompatible)
{
    VirtualRobot::TriMeshModelPtr tm2 = tm;
    if (!trimeshAlreadyCGALCompatible)
    {
        VR_INFO << "Converting tm to compatible structure" << endl;
        tm2 = SimoxCGAL::CGALMeshConverter::ConvertTrimeshCGALCompatible(tm);
        VR_INFO << "Converting tm to compatible structure...done" << endl;
    }

    VR_INFO << "Converting tm to cgal polyhedron mesh structure" << endl;


    CGALPolyhedronMeshPtr res(new CGALPolyhedronMesh(PolyhedronMeshPtr(new PolyhedronMesh())));
    CGALPolyhedronMeshBuilder b(tm2);
    PolyhedronMeshPtr mesh = res->getMesh();
    mesh->delegate(b);

    return res;
}

SimoxCGAL::CGALSurfaceMeshPtr SimoxCGAL::CGALMeshConverter::ConvertToSurfaceMesh(VirtualRobot::TriMeshModelPtr tm, bool trimeshAlreadyCGALCompatible)
{
    VirtualRobot::TriMeshModelPtr tm2 = tm;
    if (!trimeshAlreadyCGALCompatible)
    {
        VR_INFO << "Converting tm to compatible structure" << endl;
        tm2 = SimoxCGAL::CGALMeshConverter::ConvertTrimeshCGALCompatible(tm);
        VR_INFO << "Converting tm to compatible structure...done" << endl;
    }

    VR_INFO << "Converting tm to cgal surface mesh structure" << endl;

    SurfaceMeshPtr mesh(new SurfaceMesh());
/*
    std::map<Point, SurfaceMesh::vertex_index> id_map;

    for (int i = 0; i < tm2->vertices.size(); i++)
    {
        Eigen::Vector3f p = tm2->vertices.at(i);
        Point point(p[0], p[1], p[2]);
        SurfaceMesh::vertex_index index = mesh->add_vertex(point);
        id_map[point] = index;
    }*/

    std::map<unsigned int, SurfaceMesh::vertex_index> id_map;

   for (size_t f = 0; f < tm2->faces.size(); f++)
   {
       unsigned int id1 = tm2->faces.at(f).id1;
       SurfaceMesh::Vertex_index v1;
       if (id_map.find(id1) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm2->vertices.at(id1);
           Point point(p[0], p[1], p[2]);
           v1 = mesh->add_vertex(point);
           id_map[id1] = v1;
       } else
       {
           //query
           v1 = id_map[id1];
       }


       unsigned int id2 = tm2->faces.at(f).id2;
       SurfaceMesh::Vertex_index v2;
       if (id_map.find(id2) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm2->vertices.at(id2);
           Point point(p[0], p[1], p[2]);
           v2 = mesh->add_vertex(point);
           id_map[id2] = v2;
       } else
       {
           //query
           v2 = id_map[id2];
       }


       unsigned int id3 = tm2->faces.at(f).id3;
       SurfaceMesh::Vertex_index v3;
       if (id_map.find(id3) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm2->vertices.at(id3);
           Point point(p[0], p[1], p[2]);
           v3 = mesh->add_vertex(point);
           id_map[id3] = v3;
       } else
       {
           //query
           v3 = id_map[id3];
       }

       mesh->add_face(v1,v2,v3);
   }

   VR_INFO << "Converting tm to cgal data structure...done" << endl;

   CGALSurfaceMeshPtr m(new CGALSurfaceMesh(mesh));
   return m;
}

TriMeshModelPtr CGALMeshConverter::ConvertCGALMesh(CGALSurfaceMeshPtr m)
{
    VR_ASSERT(m);

    SurfaceMeshPtr mesh = m->getMesh();
    TriMeshModelPtr res(new TriMeshModel());
    std::map<size_t,size_t> idMap;
    size_t i = 0;

    BOOST_FOREACH(SurfaceMesh::vertex_index index, mesh->vertices())
    {
        Point a = mesh->point(index);
        size_t indxCGAL = index.operator size_type();
        Eigen::Vector3f v;
        v << a[0],  a[1],  a[2];
        res->addVertex(v);
        size_t indxSimox = i;
        i++;
        idMap[indxCGAL] = indxSimox;
    }
    SurfaceMesh::Vertex_around_face_iterator fb, fe;
    SurfaceMesh::Halfedge_index hi;

    BOOST_FOREACH(SurfaceMesh::face_index index , mesh->faces())
    {
        hi = mesh->halfedge(index);
        fb = mesh->vertices_around_face(hi).begin();
        fe = mesh->vertices_around_face(hi).end();
        std::vector<int> polIndex;

        for (; fb != fe; ++fb)
        {
            polIndex.push_back(fb->operator size_type());

        }
        if (polIndex.size()!=3)
        {
            VR_ERROR << "polygon with # edges!=3 is not supported: edge count " << polIndex.size() << endl;
            continue;
        }

        MathTools::TriangleFace f;
        f.id1 = polIndex.at(0);
        f.id2 = polIndex.at(1);
        f.id3 = polIndex.at(2);

        res->addFace(f);
    }
    return res;
}

TriMeshModelPtr CGALMeshConverter::ConvertCGALMesh(CGALPolyhedronMeshPtr m)
{
    VR_ASSERT(m);
    VR_ASSERT(m->getMesh());

    TriMeshModelPtr res(new TriMeshModel);
    std::map<PolyVertexConstHandle, unsigned int> pointMap;

    for (PolyhedronMesh::Facet_const_iterator facet_it = m->getMesh()->facets_begin(); facet_it != m->getMesh()->facets_end(); ++facet_it)
    {

        //access vertices
        std::vector<unsigned int> vertices;

        PolyhedronMesh::Halfedge_around_facet_const_circulator half_it = facet_it->facet_begin();

        do
        {

            if (pointMap.find(half_it->vertex()) != pointMap.end())
                vertices.push_back(pointMap.at(half_it->vertex()));
            else
            {
                PointPoly tmp_in_point = half_it->vertex()->point();
                Eigen::Vector3f tmp_out_point(tmp_in_point.x(), tmp_in_point.y(), tmp_in_point.z());
                unsigned int vid = res->addVertex(tmp_out_point);
                vertices.push_back(vid);
                pointMap[half_it->vertex()] = vid;
            }
            //PointPoly tmp_in_point = half_it->vertex()->point();
            //Eigen::Vector3f tmp_out_point(tmp_in_point.x(), tmp_in_point.y(), tmp_in_point.z());
            //vertices.push_back(tmp_out_point);
        }
        while (++half_it != facet_it->facet_begin());
        if (vertices.size() > 2)
        {
            res->addFace(vertices.at(0), vertices.at(1), vertices.at(2));
            MathTools::TriangleFace &f = res->faces.at(res->faces.size()-1);
            f.normal = TriMeshModel::CreateNormal(res->vertices.at(vertices.at(0)), res->vertices.at(vertices.at(1)), res->vertices.at(vertices.at(2)));
            //res->addTriangleWithFace(vertices.at(0), vertices.at(1), vertices.at(2));
            //TODO write to a right structure with neighboorhood relationships
        }
    }
    return res;
}

VirtualRobot::TriMeshModelPtr SimoxCGAL::CGALMeshConverter::ConvertTrimeshCGALCompatible(VirtualRobot::TriMeshModelPtr tm)
{
    VR_ASSERT (tm);

    TriMeshModelPtr result(new TriMeshModel());


    //copy data
    result->normals = tm->normals;
    result->materials = tm->materials;
    result->colors = tm->colors;

    for (std::vector<MathTools::TriangleFace>::iterator itFacesOld = tm->faces.begin(); itFacesOld != tm->faces.end(); itFacesOld++)
    {
        bool found1 = false;
        bool found2 = false;
        bool found3 = false;
        int newPos1 = -1;
        int newPos2 = -1;
        int newPos3 = -1;
        std::vector<Eigen::Vector3f>::iterator itVerticesOld = tm->vertices.begin();
        Eigen::Vector3f candidateVertex1 = *(itVerticesOld + itFacesOld->id1);
        Eigen::Vector3f candidateVertex2 = *(itVerticesOld + itFacesOld->id2);
        Eigen::Vector3f candidateVertex3 = *(itVerticesOld + itFacesOld->id3);
        for (std::vector<Eigen::Vector3f>::iterator itVerticesNew = result->vertices.begin(); itVerticesNew != result->vertices.end(); itVerticesNew++)
        {
            if ((*itVerticesNew - candidateVertex1).norm() < 0.00001)
            {
                found1 = true;
                newPos1 = std::distance(result->vertices.begin(), itVerticesNew);
            }
            if ((*itVerticesNew - candidateVertex2).norm() < 0.000001)
            {
                found2 = true;
                newPos2 = std::distance(result->vertices.begin(), itVerticesNew);
            }
            if ((*itVerticesNew - candidateVertex3).norm() < 0.000001)
            {
                found3 = true;
                newPos3 = std::distance(result->vertices.begin(), itVerticesNew);
            }
        }
        //Eventuelle füge die alten vertices zu den neuen hinzu und specheiere Position
        if (!found1)
        {
            result->addVertex(candidateVertex1);
            newPos1 = result->vertices.size() - 1;
        }
        if (!found2)
        {
            result->addVertex(candidateVertex2);
            newPos2 = result->vertices.size() - 1;
        }
        if (!found3)
        {
            result->addVertex(candidateVertex3);
            newPos3 = result->vertices.size() - 1;
        }
        //create new Face
        MathTools::TriangleFace temp;
        if ((newPos1 >= -1) && (newPos2 >= -1) && (newPos3 >= -1))
        {
            std::vector<Eigen::Vector3f>::iterator itVerticesNew = result->vertices.begin();
            //test for clockwise or counterclockwise
            Eigen::Vector3f p1p2 = *(itVerticesNew + newPos2) - *(itVerticesNew + newPos1);
            Eigen::Vector3f p3p1 = *(itVerticesNew + newPos1) - *(itVerticesNew + newPos3);
            Eigen::Vector3f cross = p1p2.cross(p3p1);
            float l = cross.norm();
            //Punkte müssen so hinzugefügt werden, das die Halfedges gegen den Uhrzeigersinn angeordnet sind, wenn das polyhedron von aussen gesehen wird
            if (l != 0)
            {
                cross /= l;
                Eigen::Vector3f diff = itFacesOld->normal - cross;
                if (diff.norm() <= 0.1)
                {
                    temp.set(newPos1, newPos3, newPos2);
                }
                else
                {
                    temp.set(newPos1, newPos2, newPos3);
                }

            }
            else
            {
                VR_INFO << "Error cross product is equal to zero" << endl;
                temp.set(newPos1, newPos3, newPos2);
            }

        }
        else
        {
            VR_ERROR << "Error in write to CgalStructure, one newPos is smaller than zero " << endl;
        }
        temp.setColor(itFacesOld->idColor1, itFacesOld->idColor2, itFacesOld->idColor3);
        temp.setNormal(itFacesOld->idNormal1, itFacesOld->idNormal2, itFacesOld->idNormal3);
        temp.normal = itFacesOld->normal;
        temp.setMaterial(itFacesOld->idMaterial);
        result->addFace(temp);
    }

    return result;
}

/*
CGALPolyhedronMeshPtr CGALMeshConverter::PolygonSoupToPolyhedronMesh(TriMeshModelPtr tm)
{
    std::vector<KernelPolyhedron::Point_3> points;
    std::vector< std::vector<std::size_t> > polygons;

    for (size_t i=0;i<tm->vertices.size();i++)
    {
        Eigen::Vector3f &r = tm->vertices.at(i);
        KernelPolyhedron::Point_3 p(r[0],r[1],r[2]);
        points.push_back(p);
    }

    for (size_t i=0;i<tm->faces.size();i++)
    {
        MathTools::TriangleFace &f = tm->faces.at(i);
        std::vector<std::size_t> pol;
        pol.push_back(f.id1);
        pol.push_back(f.id2);
        pol.push_back(f.id3);
        polygons.push_back(pol);
    }

    CGAL::Polygon_mesh_processing::orient_polygon_soup(points, polygons);
    PolyhedronMeshPtr mesh(new PolyhedronMesh);
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, polygons, *mesh);
    if (CGAL::is_closed(*mesh) && (!CGAL::Polygon_mesh_processing::is_outward_oriented(*mesh)))
      CGAL::Polygon_mesh_processing::reverse_face_orientations(*mesh);

    CGALPolyhedronMeshPtr res(new CGALPolyhedronMesh(mesh));

    return res;
}*/

}
