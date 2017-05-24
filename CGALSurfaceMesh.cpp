#include "CGALSurfaceMesh.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/rapidxml.hpp>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/XML/BaseIO.h>

using namespace VirtualRobot;

namespace SimoxCGAL {

CGALSurfaceMesh::CGALSurfaceMesh(SurfaceMeshPtr m)
    : mesh(m)
{

}


CGALSurfaceMesh::~CGALSurfaceMesh()
{

}

SurfaceMeshPtr CGALSurfaceMesh::getMesh()
{
    return mesh;
}

unsigned int CGALSurfaceMesh::getNrOfVertices()
{
    if (!mesh)
        return 0;
    return mesh->number_of_vertices();
}

unsigned int CGALSurfaceMesh::getNrOfFaces()
{
    if (!mesh)
        return 0;
    return mesh->number_of_faces();

}

unsigned int CGALSurfaceMesh::getNrOfEdges()
{
    if (!mesh)
        return 0;
    return mesh->number_of_edges();

}

void CGALSurfaceMesh::print()
{
    cout << "Simox CGAL Surface Mesh" << endl;
    if (!mesh)
    {
        cout << "No mesh data" << endl;
        return;
    }
    cout << "Nr vertices:" << getNrOfVertices() << endl;
    cout << "Nr faces:" << getNrOfFaces() << endl;

    BOOST_FOREACH(SurfaceMesh::vertex_index index, mesh->vertices())
    {
        Point a = mesh->point(index);
        cout << "Vertex " << index.operator size_type() << ":" << a[0] << ", " << a[1] << ", " << a[2] << endl;
    }


    SurfaceMesh::Vertex_around_face_iterator fb, fe;
    SurfaceMesh::Halfedge_index hi;

    BOOST_FOREACH(SurfaceMesh::face_index index , mesh->faces())
    {
        int i = 0;

        hi = mesh->halfedge(index);
        fb = mesh->vertices_around_face(hi).begin();
        fe = mesh->vertices_around_face(hi).end();

        cout << "Face " << index.operator size_type() << ":" << endl;

        cout << "\tVertexIndices:";
        for (; fb != fe; ++fb)
        {
            cout << fb->operator size_type() << ", ";
            i++;
        }
        cout << endl;
    }

}

std::string CGALSurfaceMesh::toXML(int nrTabs)
{
    std::string t;
    std::string ta = "\t";
    for (int i=0;i<nrTabs;i++)
        t += "\t";
    std::stringstream ss;

    ss << t << "<SimoxCGAL-SurfaceMesh>\n";

    ss << t <<"<NumberOfVertices vertices='" << mesh->number_of_vertices() << "'/>\n";

    ss << t <<"<NumberOfFaces faces='" << mesh->number_of_faces() << "'/>\n";

    ss << t << "<Vertices>\n";

    BOOST_FOREACH(SurfaceMesh::vertex_index index, mesh->vertices())
    {
        Point a = mesh->point(index);
        ss << t << ta << "<Vertex index='" << index.operator size_type() << "'>\n";
        ss << t << ta << ta << "<Point x='" << a[0] << "' y='" << a[1] << "' z='" << a[2] << "'/>\n";
        ss << t << ta << "</Vertex>\n";
    }

    ss << t << "</Vertices>\n";

    ss << t << "<Faces>\n";

    SurfaceMesh::Vertex_around_face_iterator fb, fe;
    SurfaceMesh::Halfedge_index hi;
    std::string value = "index";
    int i = 0;

    BOOST_FOREACH(SurfaceMesh::face_index index , mesh->faces())
    {
        i = 0;

        ss << t << ta << "<Face index='" << index.operator size_type() << "'>\n";

        hi = mesh->halfedge(index);
        fb = mesh->vertices_around_face(hi).begin();
        fe = mesh->vertices_around_face(hi).end();

        ss << t << ta << ta << "<VerticesIndex ";

        for (; fb != fe; ++fb)
        {
            std::stringstream out;
            out << value;
            out << i;
            out << "='";
            out << fb->operator size_type();
            out << "' ";
            ss << out.str();
            i++;

        }

        ss << "/>\n";

        ss << t << ta << "</Face>\n";

    }

    ss << t << ta << "</Faces>\n";

    ss << t << "</SimoxCGAL-SurfaceMesh>\n";

    return ss.str();
}

boost::shared_ptr<CGALSurfaceMesh> CGALSurfaceMesh::fromXML(const std::string &xml)
{
    CGALSurfaceMeshPtr res(new CGALSurfaceMesh(SurfaceMeshPtr(new SurfaceMesh())));
    SurfaceMeshPtr mesh = res->getMesh();

    int number_of_vertices = 0;
    int number_of_faces = 0;

    char* y = new char[xml.size() + 1];
    strncpy(y, xml.c_str(), xml.size() + 1);

    try {

        rapidxml::xml_document<char> doc;    // character type defaults to char
        doc.parse<0>(y);    // 0 means default parse flags
        rapidxml::xml_node<char>* xml = doc.first_node("SimoxCGAL-SurfaceMesh");
        THROW_VR_EXCEPTION_IF(!xml, "No <SimoxCGAL-SurfaceMesh> tag in XML definition");

        //vertices
        rapidxml::xml_node<>* tmp_number_vertices_node = xml->first_node("NumberOfVertices", 0, false);
        rapidxml::xml_attribute<>* tmp_number_vertices = tmp_number_vertices_node->first_attribute();
        number_of_vertices = BaseIO::convertToInt(tmp_number_vertices->value());

        //faces
        rapidxml::xml_node<>* tmp_number_faces_node = xml->first_node("NumberOfFaces", 0, false);
        rapidxml::xml_attribute<>* tmp_number_faces = tmp_number_faces_node->first_attribute();
        number_of_faces = BaseIO::convertToInt(tmp_number_faces->value());

        rapidxml::xml_node<>* node = xml->first_node("Vertices", 0, false);

        for (rapidxml::xml_node<>* child = node->first_node(); child != NULL; child = child->next_sibling())
        {
            rapidxml::xml_node<>* pointNode = child->first_node("Point",0,false);
            float x = BaseIO::getFloatByAttributeName(pointNode, "x");
            float y = BaseIO::getFloatByAttributeName(pointNode, "y");
            float z = BaseIO::getFloatByAttributeName(pointNode, "z");

            Point vertex = Point(x,y,z);
            mesh->add_vertex(vertex);
        }

        node = node->next_sibling("Faces", 0, false);

        for (rapidxml::xml_node<>* child = node->first_node(); child != NULL; child = child->next_sibling())
        {

            rapidxml::xml_node<>* index = child->first_node("VerticesIndex", 0, false);
            rapidxml::xml_attribute<>* index_v0 = index->first_attribute("index0", 0, false);
            rapidxml::xml_attribute<>* index_v1 = index->first_attribute("index1", 0, false);
            rapidxml::xml_attribute<>* index_v2 = index->first_attribute("index2", 0, false);

            int i_v0 = BaseIO::convertToInt(index_v0->value());
            int i_v1 = BaseIO::convertToInt(index_v1->value());
            int i_v2 = BaseIO::convertToInt(index_v2->value());

            SurfaceMesh::Vertex_index a(i_v0);
            SurfaceMesh::Vertex_index b(i_v1);
            SurfaceMesh::Vertex_index c(i_v2);

            mesh->add_face(a,b,c);
        }


        delete[] y;

    }catch (VirtualRobot::VirtualRobotException&)
    {
        // rethrow the current exception
        delete[] y;
        throw;
    }
    return res;
}

}
