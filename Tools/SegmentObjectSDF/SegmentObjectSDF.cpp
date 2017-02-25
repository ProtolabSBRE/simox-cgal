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

#include "SegmentObjectSDFWindow.h"


int main(int argc, char* argv[])
{
    SoDB::init();
    SoQt::init(argc, argv, "SegmentObjectSDF");
    cout << " --- START --- " << endl;

    std::string object("objects/LegoXWing.xml");
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

    SegmentObjectSDFWindow rw(object);

    rw.main();

    return 0;
}
