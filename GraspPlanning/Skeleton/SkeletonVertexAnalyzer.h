#ifndef SimoxCGAL_SkeletonVertexAnalyzer_H
#define SimoxCGAL_SkeletonVertexAnalyzer_H

#include "Eigen/Dense"
#include "Inventor/nodes/SoSeparator.h"
#include "SimoxCGAL.h"
#include "VirtualRobot/VirtualRobotImportExport.h"
#include "VirtualRobot/MathTools.h"
#include "Segmentation/Skeleton/SkeletonPart.h"



namespace SimoxCGAL
{

struct PrincipalAxis3D
{
    PrincipalAxis3D()
    {
        eigenvalue1 = eigenvalue2 = eigenvalue3 = 0.0f;
        t1 = t2 = 0.0f;
    }

    Eigen::Vector3f pca1;
    Eigen::Vector3f pca2;
    Eigen::Vector3f pca3;

    float eigenvalue1;
    float eigenvalue2;
    float eigenvalue3;

    float t1;
    float t2;
};

struct Diameter
{
    Diameter()
    {
        minDiameter = maxDiameter = averageDiameter = 0.0f;
    }

    float minDiameter;
    float maxDiameter;
    float averageDiameter;

    void print() {
        std::cout << "Diameter: ";
        std::cout << " //tmin: " << minDiameter;
        std::cout << " //tmax: " << maxDiameter;
        std::cout << " //average: " << averageDiameter << std::endl;
    }

};

struct SkeletonVertexResult
{
    SkeletonVertexResult()
    {
       indexVertex=-1;
       endpoint = false;
       valid = false;
    }

    SimoxCGAL::SkeletonPtr skeleton;
    SimoxCGAL::SkeletonPartPtr part;
    PrincipalAxis3D pca;
    VirtualRobot::MathTools::Plane graspingPlane;
    SkeletonPointPtr skeletonPoint; // center
    std::vector<SkeletonVertex> interval;
    int indexVertex;
    bool endpoint;
    bool valid;
};

class SkeletonVertexAnalyzer
{
public:
    static SkeletonVertexResult calculatePCA(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, int indexVertex, SimoxCGAL::SkeletonPartPtr part, float length, bool verbose);


    static bool calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f& dir1, Eigen::Vector3f& dir2, Eigen::Vector3f& result);
    static Eigen::Vector3f pointToVector(SimoxCGAL::Point point);
    static std::vector<Eigen::Vector3f> projectPointsToPlane(std::vector<VirtualRobot::MathTools::Plane> v_planes, std::vector<std::vector<Eigen::Vector3f>> v_points);
    static Eigen::Vector3f createMidVector(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2);
    static Diameter calculateDiameter(Eigen::Vector3f &pos, std::vector<Eigen::Vector3f> &points);
    static Diameter getPlanesWithMeshPoints(const SimoxCGAL::SkeletonPtr skeleton, const SimoxCGAL::SurfaceMeshPtr mesh, const std::vector<SimoxCGAL::SkeletonVertex> &interval, const VirtualRobot::MathTools::Plane &splane, std::vector<Eigen::Vector3f> &storePoints);



protected:

    SkeletonVertexAnalyzer();
    ~SkeletonVertexAnalyzer();
};

}

#endif
