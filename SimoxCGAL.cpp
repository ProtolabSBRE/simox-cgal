
#include "SimoxCGAL.h"
#include <VirtualRobot/RuntimeEnvironment.h>

namespace SimoxCGAL
{

std::string globalAppName;

void init(const std::string &appName)
{
    VirtualRobot::init(appName);
    char* simox_cgal_data_path = getenv("SIMOX_CGAL_DATA_PATH");
    if (simox_cgal_data_path)
    {
        VirtualRobot::RuntimeEnvironment::addDataPath(std::string(simox_cgal_data_path), true);
    }
#ifdef Simox_CGAL_DATA_PATH
            VirtualRobot::RuntimeEnvironment::addDataPath(std::string(Simox_CGAL_DATA_PATH), true);
#endif
}

void init(int &argc, char* argv[], const std::string &appName)
{
    VirtualRobot::init(argc, argv, appName);

    char* simox_cgal_data_path = getenv("SIMOX_CGAL_DATA_PATH");
    if (simox_cgal_data_path)
    {
        VirtualRobot::RuntimeEnvironment::addDataPath(std::string(simox_cgal_data_path), true);
    }
#ifdef Simox_CGAL_DATA_PATH
            VirtualRobot::RuntimeEnvironment::addDataPath(std::string(Simox_CGAL_DATA_PATH), true);
#endif
}




}
