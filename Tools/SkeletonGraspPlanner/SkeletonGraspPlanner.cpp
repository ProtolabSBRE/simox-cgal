#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SkeletonGraspPlannerWindow.h"


int main(int argc, char* argv[])
{
    SimoxCGAL::init(argc, argv, "Skeleton Grasp Planner");
    std::string robot;

    //Gripper's path file
    std::string end_effector;
    end_effector = "LGripper";
    robot = "robots/Pepper/LastVersion/" + end_effector + ".xml";
    //Name of the end effector
    std::string eef(end_effector);
    //Object path file
    std::string object("objects/similar-objects/newObjects/cube6cm_6146v.soxml");

    // robot = "robots/ArmarIII/ArmarIII.xml";
    // std::string eef("Hand R");
    // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"

//    std::string robot("robots/Shadow_Dexterous_Hand/shadowhand.xml");
      // robot = "robots/iCub/iCub.xml";
//    std::string robot("robots/SAH_RightHand/SAH_RightHand.xml");
//    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);

    //std::string object("objects/wok.xml");
    //std::string object("objects/riceBox.xml");
    //std::string object("objects/screwdriver/screwdriver01.xml");
    // std::string object("segmented-objects/flashlight/flashlight1.soxml");
    // std::string object("objects/similar-objects/airplane/airplane0.soxml");

    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);
    //std::string preshape("");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);

    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

    if (!robFile.empty())
    {
        if(VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
            robot = robFile;
        else
            VR_ERROR << "Could not find file " << robFile;
    }

    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");
    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);

    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");
    if (!eefname.empty())
    {
        eef = eefname;
    }

    cout << "Robot file (--robot <filename>): " << robot << endl;
    cout << "End effector (--endeffector <eef-name>): " << eef << endl;
    cout << "Object file (--object <object-file>: " << object << endl;
    cout << "-----------------" << endl;

    SkeletonGraspPlannerWindow rw(robot, eef, object);

    rw.main();

    return 0;
}
