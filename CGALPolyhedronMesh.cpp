#include "CGALPolyhedronMesh.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/XML/rapidxml.hpp>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/XML/BaseIO.h>

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>

#include "CGALMeshConverter.h"

using namespace VirtualRobot;

namespace SimoxCGAL {

CGALPolyhedronMesh::CGALPolyhedronMesh(PolyhedronMeshPtr m)
    : mesh(m)
{

}


CGALPolyhedronMesh::~CGALPolyhedronMesh()
{

}

PolyhedronMeshPtr CGALPolyhedronMesh::getMesh()
{
    return mesh;
}

unsigned int CGALPolyhedronMesh::getNrOfVertices()
{
    if (!mesh)
        return 0;
    return mesh->size_of_vertices();
}

unsigned int CGALPolyhedronMesh::getNrOfFaces()
{
    if (!mesh)
        return 0;
    return mesh->size_of_facets();

}

unsigned int CGALPolyhedronMesh::getNrOfEdges()
{
    if (!mesh)
        return 0;
    return mesh->size_of_halfedges();

}

void CGALPolyhedronMesh::print()
{
    cout << "Simox CGAL Polyhedron Mesh" << endl;
    if (!mesh)
    {
        cout << "No mesh data" << endl;
        return;
    }
    cout << "Nr vertices:" << getNrOfVertices() << endl;
    cout << "Nr faces:" << getNrOfFaces() << endl;

    int a = 0;

    for ( PolyhedronMesh::Vertex_iterator v = mesh->vertices_begin(); v != mesh->vertices_end(); ++v)
    {
        cout << "Vertex " << a << ":" << v->point() << endl;
        a++;
    }

    a=0;
    for ( PolyFacetIterator i = mesh->facets_begin(); i != mesh->facets_end(); ++i)
    {
        cout << "Face " << a << ":" << endl;

        cout << "\tVertexIndices:";

        PolyHalfedgeFacetCirculator j = i->facet_begin();
        do {
            std::cout << ' ' << std::distance(mesh->vertices_begin(), j->vertex());
        } while ( ++j != i->facet_begin());

        cout << endl;
    }

}

std::string CGALPolyhedronMesh::toXML(int nrTabs)
{
    std::string t;
    std::string ta = "\t";
    for (int i=0;i<nrTabs;i++)
        t += "\t";
    std::stringstream ss;

    ss << t << "<SimoxCGAL-PolyhedronMesh>\n";

    ss << t << ta << "<NumberOfVertices vertices='" << getNrOfVertices() << "'/>\n";

    ss << t << ta << "<NumberOfFaces faces='" << getNrOfFaces() << "'/>\n";

    ss << t <<  ta << "<Vertices>\n";
    int k=0;
    for ( PolyhedronMesh::Vertex_iterator v = mesh->vertices_begin(); v != mesh->vertices_end(); ++v)
    {
        PointPoly a = v->point();
        ss << t << ta << ta  << "<Vertex index='" << k << "'>\n";
        ss << t << ta << ta  << ta << "<Point x='" << a[0] << "' y='" << a[1] << "' z='" << a[2] << "'/>\n";
        ss << t << ta << ta  << "</Vertex>\n";
        k++;
    }

    ss << t << ta << "</Vertices>\n";

    ss << t << ta << "<Faces>\n";

    std::string value = "index";
    int i = 0;
    k=0;
    for ( PolyFacetIterator it = mesh->facets_begin(); it != mesh->facets_end(); ++it)
    {
        i = 0;
        ss << t << ta <<  ta << "<Face index='" << k << "'>\n";
        PolyHalfedgeFacetCirculator j = it->facet_begin();
        ss << t << ta << ta <<  ta << "<VerticesIndex ";

        do
        {
            std::stringstream out;
            out << value;
            out << i;
            out << "='";
            out << std::distance(mesh->vertices_begin(), j->vertex());
            out << "' ";
            ss << out.str();
            i++;

        } while ( ++j != it->facet_begin());

        ss << "/>\n";

        ss << t << ta <<  ta << "</Face>\n";

    }

    ss << t << ta << "</Faces>\n";

    ss << t << "</SimoxCGAL-PolyhedronMesh>\n";

    return ss.str();
}

boost::shared_ptr<CGALPolyhedronMesh> CGALPolyhedronMesh::fromXML(const std::string &xml)
{
    CGALPolyhedronMeshPtr res(new CGALPolyhedronMesh(PolyhedronMeshPtr(new PolyhedronMesh())));
    VirtualRobot::TriMeshModelPtr tm(new VirtualRobot::TriMeshModel());

    int number_of_vertices = 0;
    int number_of_faces = 0;

    char* y = new char[xml.size() + 1];
    strncpy(y, xml.c_str(), xml.size() + 1);

    try {

        rapidxml::xml_document<char> doc;    // character type defaults to char
        doc.parse<0>(y);    // 0 means default parse flags
        rapidxml::xml_node<char>* xml = doc.first_node("SimoxCGAL-PolyhedronMesh");
        THROW_VR_EXCEPTION_IF(!xml, "No <SimoxCGAL-PolyhedronMesh> tag in XML definition");

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
            //Point vertex = Point(x,y,z);
            Eigen::Vector3f v;
            v << x,y,z;
            tm->addVertex(v);
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

            VirtualRobot::MathTools::TriangleFace f;
            f.id1 = i_v0;
            f.id2 = i_v1;
            f.id3 = i_v2;
            tm->addFace(f);

            /*
            PolyhedronMesh::Vertex_index a(i_v0);
            PolyhedronMesh::Vertex_index b(i_v1);
            PolyhedronMesh::Vertex_index c(i_v2);
            mesh->add_face(a,b,c);
            */
        }

        res = CGALMeshConverter::ConvertToPolyhedronMesh(tm, true);
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
