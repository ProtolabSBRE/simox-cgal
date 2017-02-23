#include "MeshSDF.h"

using namespace VirtualRobot;

namespace SimoxCGAL {

MeshSDF::MeshSDF(CGALSurfaceMeshPtr mesh)
    :mesh(mesh)
{
    VR_ASSERT(mesh);

    buildSkeleton();
}

MeshSDF::~MeshSDF()
{

}

bool MeshSDF::buildSkeleton()
{

}

}
