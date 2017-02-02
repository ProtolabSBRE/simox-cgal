/**
* @package    SimoxCGAL
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE SimoxCGAL_SimoxCGALMeshTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SimoxCGAL.h"
#include "CGALMeshConverter.h"
using namespace VirtualRobot;
using namespace SimoxCGAL;

BOOST_AUTO_TEST_SUITE(SimoxCGALMesh)


BOOST_AUTO_TEST_CASE(testMesh)
{
    TriMeshModelPtr tm(new TriMeshModel());
    Eigen::Vector3f v1;
    v1 << 0,0,0;
    Eigen::Vector3f v2;
    v2 << 0,0,1;
    Eigen::Vector3f v3;
    v3 << 1,0,0;
    Eigen::Vector3f v4;
    v4 << 0,1,0;
    tm->addTriangleWithFace(v1,v2,v3);
    tm->addTriangleWithFace(v1,v2,v4);
    tm->addTriangleWithFace(v1,v3,v4);
    tm->addTriangleWithFace(v2,v3,v4);
    BOOST_REQUIRE(tm);


    CGALMeshPtr tmc = CGALMeshConverter::ConvertTrimesh(tm);
    BOOST_REQUIRE(tmc);
    BOOST_REQUIRE(tmc->getMesh());

    // todo: check vertices, faces

    // todo:
    //TriMeshModelPtr tm2 = CGALMeshConverter::ConvertCGALMesh(tmc);
    //BOOST_REQUIRE(tm2);

    // todo: check vertices of tm2
}

BOOST_AUTO_TEST_SUITE_END()
