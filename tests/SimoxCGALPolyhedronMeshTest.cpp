/**
* @package    SimoxCGAL
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE SimoxCGAL_SimoxCGALPolyhedronMeshTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "SimoxCGAL.h"
#include "IO/CGALMeshIO.h"
#include "CGALMeshConverter.h"

using namespace VirtualRobot;
using namespace SimoxCGAL;

BOOST_AUTO_TEST_SUITE(SimoxCGALPolyhedronMesh)


BOOST_AUTO_TEST_CASE(testPolyhedronMesh)
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

    // vertex direction indicates normal!
    tm->addTriangleWithFace(v1,v2,v3);
    tm->addTriangleWithFace(v1,v4,v2);
    tm->addTriangleWithFace(v1,v3,v4);
    tm->addTriangleWithFace(v2,v4,v3);

    const int nrVertices = 4;
    const int nrFaces = 4;
    BOOST_REQUIRE(tm);

    CGALPolyhedronMeshPtr tmc = CGALMeshConverter::ConvertToPolyhedronMesh(tm);
    BOOST_REQUIRE(tmc);
    BOOST_REQUIRE(tmc->getMesh());
    BOOST_REQUIRE_EQUAL(tmc->getNrOfVertices(), nrVertices);
    BOOST_REQUIRE_EQUAL(tmc->getNrOfFaces(), nrFaces);

    // todo: check vertices, faces

    TriMeshModelPtr tm2 = CGALMeshConverter::ConvertCGALMesh(tmc);
    BOOST_REQUIRE(tm2);

    // todo: check vertices of tm2

    TriMeshModelPtr tm3 = SimoxCGAL::CGALMeshConverter::ConvertTrimeshCGALCompatible(tm);
    BOOST_REQUIRE_EQUAL(tm3->vertices.size(), nrVertices);
    BOOST_REQUIRE_EQUAL(tm3->faces.size(), nrFaces);

    CGALPolyhedronMeshPtr tmc3 = CGALMeshConverter::ConvertToPolyhedronMesh(tm3,true);
    BOOST_REQUIRE(tmc3);
    BOOST_REQUIRE(tmc3->getMesh());
    BOOST_REQUIRE_EQUAL(tmc3->getNrOfVertices(), nrVertices);
    BOOST_REQUIRE_EQUAL(tmc3->getNrOfFaces(), nrFaces);
}

BOOST_AUTO_TEST_CASE(testPolyhedronMeshIO)
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
    // vertex direction indicates normal!
    unsigned int id1 = tm->addVertex(v1);
    unsigned int id2 = tm->addVertex(v2);
    unsigned int id3 = tm->addVertex(v3);
    unsigned int id4 = tm->addVertex(v4);
    tm->addFace(id1,id2,id3);
    tm->addFace(id1,id4,id2);
    tm->addFace(id1,id3,id4);
    tm->addFace(id2,id4,id3);

    BOOST_REQUIRE(tm);

    CGALPolyhedronMeshPtr tmc = CGALMeshConverter::ConvertToPolyhedronMesh(tm);
    BOOST_REQUIRE(tmc);
    BOOST_REQUIRE(tmc->getMesh());

    std::string fn("testmeshfile-polyhedron.tmp.xml");
    bool ok = CGALMeshIO::Save(tmc,fn);
    BOOST_REQUIRE_EQUAL(ok, true);

    CGALPolyhedronMeshPtr tmc2 = CGALMeshIO::LoadPolyhedronMesh(fn);
    BOOST_REQUIRE(tmc2);
    TriMeshModelPtr tm2 = CGALMeshConverter::ConvertCGALMesh(tmc2);
    BOOST_REQUIRE(tm2);
    
    BOOST_REQUIRE_EQUAL(tm2->vertices.size(), 4);
    BOOST_REQUIRE_EQUAL(tm2->faces.size(), 4);

}

BOOST_AUTO_TEST_SUITE_END()
