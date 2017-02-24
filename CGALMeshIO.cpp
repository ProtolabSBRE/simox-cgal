#include "CGALMeshIO.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/XML/BaseIO.h>

using namespace VirtualRobot;

namespace SimoxCGAL {


CGALSurfaceMeshPtr CGALMeshIO::LoadSurfaceMesh(const std::string &filename)
{
    std::ifstream in(filename.c_str());
    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << filename);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string objectXML(buffer.str());
    in.close();

    return CGALSurfaceMesh::fromXML(objectXML);
}

CGALPolyhedronMeshPtr CGALMeshIO::LoadPolyhedronMesh(const std::string &filename)
{
    std::ifstream in(filename.c_str());
    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << filename);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string objectXML(buffer.str());
    in.close();

    return CGALPolyhedronMesh::fromXML(objectXML);
}

bool CGALMeshIO::Save(CGALSurfaceMeshPtr o, const std::string &filename)
{
    if (!o)
        return false;

    std::string xml = o->toXML();
    bool res = BaseIO::writeXMLFile(filename, xml, true);
    return res;
}

bool CGALMeshIO::Save(CGALPolyhedronMeshPtr o, const std::string &filename)
{
    if (!o)
        return false;

    std::string xml = o->toXML();
    bool res = BaseIO::writeXMLFile(filename, xml, true);
    return res;
}

}
