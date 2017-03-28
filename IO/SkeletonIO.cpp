#include "SkeletonIO.h"

#include "VirtualRobot/XML/rapidxml.hpp"
#include "VirtualRobot/XML/BaseIO.h"

using namespace rapidxml;
using namespace VirtualRobot;
using namespace SimoxCGAL;

namespace SimoxCGAL {

SkeletonIO::SkeletonIO()
{

}

SkeletonIO::~SkeletonIO()
{

}

SkeletonPtr SkeletonIO::loadSkeleton(const std::string &file)
{
    // load file
    std::ifstream in(file.c_str());
    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << file);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string objectXML(buffer.str());
    in.close();


    char* y = new char[objectXML.size() + 1];
    strncpy(y, objectXML.c_str(), objectXML.size() + 1);

    rapidxml::xml_document<char> doc;    // character type defaults to char
    doc.parse<0>(y);    // 0 means default parse flags
    rapidxml::xml_node<char>* objectXMLFile = doc.first_node("CGAL-Skeleton");
    return createSkeletonObject(objectXMLFile);


}


SkeletonPtr SkeletonIO::createSkeletonObject(rapidxml::xml_node<char> *skeletonNode)
{

    std::map<Skeleton::vertex_descriptor, Point> debug_vertices;
    int number_of_vertices = 0;
    int number_of_edges = 0;

    std::map<int, Skeleton::vertex_descriptor> id_vertices;

    cout << "\tSkeleton" << endl;

    //vertices
    rapidxml::xml_node<>* tmp_number_vertices_node = skeletonNode->first_node("NumberOfVertices", 0, false);
    rapidxml::xml_attribute<>* tmp_number_vertices = tmp_number_vertices_node->first_attribute();
    number_of_vertices = BaseIO::convertToInt(tmp_number_vertices->value());

    //edges
    rapidxml::xml_node<>* tmp_number_edges_node = skeletonNode->first_node("NumberOfEdges", 0, false);
    rapidxml::xml_attribute<>* tmp_number_edges = tmp_number_edges_node->first_attribute("edges", 0, false);
    number_of_edges = BaseIO::convertToInt(tmp_number_edges->value());


    Skeleton skeleton;
    std::vector<SurfaceMeshVertexDescriptor> input_vertex_desriptor;

    //erst alle vertices auslesen!
    rapidxml::xml_node<>* node = skeletonNode->first_node("Vertices", 0, false);

    for (rapidxml::xml_node<>* child = node->first_node(); child != NULL; child = child->next_sibling())
    {
        Skeleton::vertex_descriptor vd = boost::add_vertex(skeleton);

        rapidxml::xml_attribute<>* index_char = child->first_attribute("index", 0, false);
        int index = BaseIO::convertToInt(index_char->value());


        rapidxml::xml_node<>* coord_node = child->first_node("Coordinate",0,false);
        float x = BaseIO::getFloatByAttributeName(coord_node, "x");
        float y = BaseIO::getFloatByAttributeName(coord_node, "y");
        float z = BaseIO::getFloatByAttributeName(coord_node, "z");

        rapidxml::xml_node<>* index_mesh = child->first_node("IndexToMesh", 0 ,false);


        for (rapidxml::xml_node<>* index_to_mesh = index_mesh->first_node(); index_to_mesh != NULL; index_to_mesh = index_to_mesh->next_sibling())
        {
            rapidxml::xml_attribute<>* index_char = index_to_mesh->first_attribute("index", 0, false);
            int indexMesh = BaseIO::convertToInt(index_char->value());
            SurfaceMesh::Vertex_index i(indexMesh);
            input_vertex_desriptor.push_back(i);

        }

        //alle indexe einfügen!
        //in Skeleton einfügen
        Point vertex = Point(x,y,z);
        //Skeleton::vertex_descriptor vd = boost::add_vertex(skeleton);
        skeleton[vd].point = vertex;
        skeleton[vd].vertices = input_vertex_desriptor;

        input_vertex_desriptor.clear();

        debug_vertices[vd] = vertex;
        id_vertices[index] = vd;

    }

    node = node->next_sibling("Edges", 0, false);

    for (rapidxml::xml_node<>* child = node->first_node(); child != NULL; child = child->next_sibling())
    {
        rapidxml::xml_attribute<>* from_char = child->first_attribute("from",0,false);
        int from_index = BaseIO::convertToInt(from_char->value());
        Skeleton::vertex_descriptor from = id_vertices[from_index];

        rapidxml::xml_attribute<>* to_char = child->first_attribute("to",0,false);
        int to_index = BaseIO::convertToInt(to_char->value());
        Skeleton::vertex_descriptor to = id_vertices[to_index];

        //in Skeleton einfügen
        boost::add_edge(from, to, skeleton);

    }

    cout << "\tvertices_number: should be ( " << number_of_vertices << " ), actual : " << boost::num_vertices(skeleton) << endl;
    cout << "\tedges_number: should be (" << number_of_edges << "), actual: " << boost::num_edges(skeleton) << endl;
    cout << "\n";

    return SkeletonPtr(new Skeleton(skeleton));
}

bool SkeletonIO::saveSkeletonObject(CGALSkeletonPtr s, const std::string &filename)
{

    if (!s)
        return false;

    std::string xml = s->toXML();
    bool res = BaseIO::writeXMLFile(filename, xml, true);
    return res;
}

}
