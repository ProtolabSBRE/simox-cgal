#include "CGALMeshConverter.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace VirtualRobot;

namespace SimoxCGAL {

SimoxCGAL::CGALMeshPtr SimoxCGAL::CGALMeshConverter::ConvertTrimesh(VirtualRobot::TriMeshModelPtr tm, bool trimeshAlreadyCGALCompatible)
{
    if (!trimeshAlreadyCGALCompatible)
    {
        VR_INFO << "Converting tm to compatible structure" << endl;
        tm = SimoxCGAL::CGALMeshConverter::ConvertTrimeshCGALCompatible(tm);
        VR_INFO << "Converting tm to compatible structure...done" << endl;
    }

    VR_INFO << "Converting tm to cgal data structure" << endl;


    TriangleMeshPtr mesh(new TriangleMesh());
/*
    std::map<Point, TriangleMesh::vertex_index> id_map;

    for (int i = 0; i < tm->vertices.size(); i++)
    {
        Eigen::Vector3f p = tm->vertices.at(i);
        Point point(p[0], p[1], p[2]);
        TriangleMesh::vertex_index index = mesh->add_vertex(point);
        id_map[point] = index;
    }*/

    std::map<unsigned int, TriangleMesh::vertex_index> id_map;

   for (size_t f = 0; f < tm->faces.size(); f++)
   {
       unsigned int id1 = tm->faces.at(f).id1;
       TriangleMesh::Vertex_index v1;
       if (id_map.find(id1) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm->vertices.at(id1);
           Point point(p[0], p[1], p[2]);
           v1 = mesh->add_vertex(point);
           id_map[id1] = v1;
       } else
       {
           //query
           v1 = id_map[id1];
       }


       unsigned int id2 = tm->faces.at(f).id2;
       TriangleMesh::Vertex_index v2;
       if (id_map.find(id2) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm->vertices.at(id2);
           Point point(p[0], p[1], p[2]);
           v2 = mesh->add_vertex(point);
           id_map[id2] = v2;
       } else
       {
           //query
           v2 = id_map[id2];
       }


       unsigned int id3 = tm->faces.at(f).id3;
       TriangleMesh::Vertex_index v3;
       if (id_map.find(id3) == id_map.end())
       {
           // insert
           Eigen::Vector3f p = tm->vertices.at(id3);
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

   CGALMeshPtr m(new CGALMesh(mesh));
   return m;
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
            if ((*itVerticesNew - candidateVertex1).norm() < 0.01)  //check if New Vertices are exiting
            {
                found1 = true;
                newPos1 = std::distance(result->vertices.begin(), itVerticesNew);
            }
            if ((*itVerticesNew - candidateVertex2).norm() < 0.01)
            {
                found2 = true;
                newPos2 = std::distance(result->vertices.begin(), itVerticesNew);
            }
            if ((*itVerticesNew - candidateVertex3).norm() < 0.01)
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

}
