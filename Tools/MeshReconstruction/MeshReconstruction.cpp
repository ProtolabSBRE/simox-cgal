#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "MeshReconstructionWindow.h"


int main(int argc, char* argv[])
{
    SimoxCGAL::init(argc, argv, "MeshReconstruction");


    //std::string object("objects/similar-objects/airplane/airplane0.xml");
    std::string object("objects/similar-objects/flashlight/flashlight1.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);

    VirtualRobot::RuntimeEnvironment::considerKey("object");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");
    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }

    cout << "Using object from " << object << endl;
    cout << "-----------------" << endl;

    MeshReconstructionWindow rw(object);

    rw.main();

    return 0;
}
