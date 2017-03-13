#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SkeletonGraspPlanerWindow.h"


int main(int argc, char* argv[])
{
    VirtualRobot::init(argc, argv, "Simox Grasp Planner");
    cout << " --- START --- " << endl;

    // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"
    std::string robot("robots/ArmarIII/ArmarIII.xml");
//    std::string robot("robots/Shadow_Dexterous_Hand/shadowhand.xml");
//    std::string robot("robots/iCub/iCub.xml");
//    std::string robot("robots/SAH_RightHand/SAH_RightHand.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);
    std::string eef("Hand R");
    //std::string object("objects/wok.xml");
    //std::string object("objects/riceBox.xml");
    std::string object("/common/homes/students/koch/Dokumente/ba_eduard_koch/objects/wrench_strange/gut/wrench3.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);
    std::string preshape("");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::considerKey("preshape");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            robot = robFile;
        }
    }

//    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

//    if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
//    {
//        robot = robFile;
//    }

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }

    VirtualRobot::RuntimeEnvironment::addKeyValuePair("endeffector2", "SHADOWHAND");
    VirtualRobot::RuntimeEnvironment::addKeyValuePair("endeffector3", "Right Hand");
    //key1: endeffector für Armarhand
    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");

    if (!eefname.empty())
    {
        eef = eefname;
    }

    std::string ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");

    if (!ps.empty())
    {
        preshape = ps;
    }


    std::vector<std::string> path = VirtualRobot::RuntimeEnvironment::getDataPaths();

    for (int i = 0; i < path.size(); i++)
    {
        std::cout << "path:" << path.at(i) << std::endl;
    }

//    robot = "/common/homes/students/koch/Dokumente/simox/VirtualRobot/data/robots/Schunk_SAH/SAH_RightHand.xml";
    cout << "Using robot from " << robot << endl;
    cout << "End effector:" << eef << ", preshape:" << preshape << endl;
    cout << "Using object from " << object << endl;

    SkeletonGraspPlanerWindow rw(robot, eef, preshape, object);

    rw.main();

    return 0;
}
