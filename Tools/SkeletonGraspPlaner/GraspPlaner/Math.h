#ifndef MATH_H
#define MATH_H

#include "Eigen/Dense"
#include "Inventor/nodes/SoSeparator.h"
#include "SimoxCGAL.h"
#include "VirtualRobot/VirtualRobotImportExport.h"
#include "VirtualRobot/MathTools.h"
#include "Segmentation/Skeleton/SkeletonPart.h"


struct PrincipalAxis3D{
    Eigen::Vector3f pca1;
    Eigen::Vector3f pca2;
    Eigen::Vector3f pca3;

    float eigenvalue1;
    float eigenvalue2;
    float eigenvalue3;

    float t1;
    float t2;
};

struct Diameter{
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

class Math
{

public:


    static bool calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f& dir1, Eigen::Vector3f& dir2, Eigen::Vector3f& result);
    static Eigen::Vector3f pointToVector(SimoxCGAL::Point point);
    static std::vector<Eigen::Vector3f> projectPointsToPlane(std::vector<VirtualRobot::MathTools::Plane> v_planes, std::vector<std::vector<Eigen::Vector3f>> v_points);
    static bool calculatePCA(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh, const int &indexVertex, SimoxCGAL::SkeletonPartPtr part, const float &length, PrincipalAxis3D &pca, VirtualRobot::MathTools::Plane &plane);
    static Eigen::Vector3f createMidVector(const Eigen::Vector3f &vec1, const Eigen::Vector3f &vec2);
    static Diameter calculateDiameter(Eigen::Vector3f &pos, std::vector<Eigen::Vector3f> &points);
    static Diameter getPlanesWithMeshPoints(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SurfaceMeshPtr mesh,/* SubpartPtr &subpart, */std::vector<SimoxCGAL::SkeletonVertex> &interval, VirtualRobot::MathTools::Plane &splane, std::vector<Eigen::Vector3f> &storePoints);



protected:

    Math();
    ~Math();
};

#endif // MATH_H
