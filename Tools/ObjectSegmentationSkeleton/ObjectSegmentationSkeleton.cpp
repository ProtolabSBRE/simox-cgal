#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include "SimoxCGAL.h"

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "SimoxCGAL.h"
#include "ObjectSegmentationSkeletonWindow.h"


int main(int argc, char* argv[])
{
    SimoxCGAL::init(argc, argv, "ObjectSegmentationSkeleton");
    cout << " --- START --- " << endl;

    std::string object("objects/similar-objects/flashlight/flashlight1.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);
  
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }

    cout << "Object file " << object << endl;

    ObjectSegmentationSkeletonWindow rw(object);

    rw.main();

    return 0;
}
