#include "CGALMesh.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace VirtualRobot;

namespace SimoxCGAL {

CGALMesh::CGALMesh(TriangleMeshPtr m)
    : mesh(m)
{

}


CGALMesh::~CGALMesh()
{

}

TriangleMeshPtr CGALMesh::getMesh()
{
    return mesh;
}

}
