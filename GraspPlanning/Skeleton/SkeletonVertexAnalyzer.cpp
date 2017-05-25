#include "SkeletonVertexAnalyzer.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

using namespace std;
using namespace Eigen;
using namespace VirtualRobot;

namespace SimoxCGAL
{
/*!
 * \brief SkeletonVertexAnalyzer::calculateApproachDir
 *
 *  Berechnet die "Mitte" der beiden Vektoren. Dieser wird später für die Ebene als Normalenvektor verwendent.
 *  Die Methode sollte mit gültigen dir1 und dir2 aufgerufen werden! Die Richtungen sollten im lokalen Koordinatensystem
 *  definiert sein. Der Ursprung sollte der centerPoint sein.
 *
 * \param dir1      1.Nachbar vom Greifpunkt
 * \param dir2      2.Nachbar vom Greifpunkt
 * \param result    Vektor der eine Ebene erzeut die in der Mitte ist
 * \param sep       Debug (nicht wichtig)
 * \return          true, wenn ein Vektor gefunden wurde, false otherwise
 */

bool SkeletonVertexAnalyzer::calculateApproachPlane(Eigen::Vector3f &pos, Eigen::Vector3f &dir1, Eigen::Vector3f &dir2, Eigen::Vector3f &result)
{

    Eigen::Vector3f d1 = dir1 - pos;
    d1.normalize();
    Eigen::Vector3f d2 = dir2 - pos;
    d2.normalize();

    VirtualRobot::MathTools::Plane plane1;
    VirtualRobot::MathTools::Plane plane2;
    plane1.p = pos;
    plane2.p = pos;
    plane1.n = d1;
    plane2.n = d2;


    Eigen::Vector3f mid((d1[0]+d2[0])/2.f, (d1[1]+d2[1])/2.f, (d1[2]+d2[2])/2.f);

    if (mid.norm() < 0.001f)
    {
        result = d1;
        return true;
    }

    VirtualRobot::MathTools::Line line = VirtualRobot::MathTools::intersectPlanes(plane1, plane2);

    result = (line.d).cross(mid);
    result.normalize();
    return true;
}

std::vector<Eigen::Vector3f> SkeletonVertexAnalyzer::projectPointsToPlane(std::vector<VirtualRobot::MathTools::Plane> v_planes, std::vector<std::vector<Eigen::Vector3f> > v_points)
{
    VirtualRobot::MathTools::Plane plane = v_planes.at(0);

    std::vector<Eigen::Vector3f> result;

    for (size_t i = 0; i < v_points.size(); i++)
    {
        for (size_t j = 0; j < v_points.at(i).size(); j++)
        {
            Eigen::Vector3f p = VirtualRobot::MathTools::projectPointToPlane(v_points.at(i).at(j), plane);
            result.push_back(p);
        }
    }

    return result;


}

SkeletonVertexResult SkeletonVertexAnalyzer::calculatePCA(SkeletonPtr skeleton, SurfaceMeshPtr mesh, int indexVertex, SkeletonPartPtr part, float length, bool verbose)
{
    SkeletonVertexResult result;
    result.valid = false;
    result.skeleton = skeleton;
    result.part = part;
    result.indexVertex = indexVertex;

    //vector<SkeletonVertex> interval;
    vector<Vector3f> points;

    unsigned long indx = part->sortedSkeletonPartIndex.at(indexVertex);
    SkeletonPointPtr point = part->skeletonPart[indx];
    result.skeletonPoint = point;

    bool endpoint = point->endpoint;
    result.endpoint = endpoint;

    bool valid = part->calculateInterval(skeleton, indexVertex, length, endpoint, result.interval, verbose);

    if (!valid)
    {
        if (verbose)
            VR_INFO << "Intervall not valid" << endl;
        return result;
    }

    //result.interval = interval;
    Point pos = (*skeleton)[result.interval.at(0)].point; // indexVertex point is first entry of interval
    Eigen::Vector3f posE(pos[0], pos[1], pos[2]);

    result.graspingPlane.p = posE;

    if (endpoint)
    {
        Point end = (*skeleton)[point->neighbor.front()].point;
        result.graspingPlane.n = Eigen::Vector3f(end[0] - pos[0], end[1] - pos[1], end[2] - pos[2]);
        result.graspingPlane.n.normalize();

    } else {

        Point n1 = (*skeleton)[point->neighbor.front()].point;
        Point n2 = (*skeleton)[point->neighbor.back()].point;

        Eigen::Vector3f n1e(n1[0], n1[1], n1[2]);
        Eigen::Vector3f n2e(n2[0], n2[1], n2[2]);

        calculateApproachPlane(result.graspingPlane.p, n1e, n2e, result.graspingPlane.n);
    }


    getPlanesWithMeshPoints(skeleton, mesh, result.interval, result.graspingPlane, points);

    if (points.size() == 0)
    {
        VR_ERROR << "zero points?!" << endl;
        return result;
    }

    int nrPoints = (int)points.size() - 1; // -1?




    Eigen::Vector3f mean(0.f, 0.f, 0.f);

    for (size_t i = 0; i < points.size(); i++)
    {
        mean += points.at(i);
    }

    mean /= nrPoints;

    Eigen::MatrixXf matrix(points.size(), 3);

    for (size_t i  = 0; i < points.size(); i++)
    {
        points.at(i) -= mean;
        matrix.block(i, 0, 1, 3) = points.at(i).transpose();
    }


    Eigen::JacobiSVD<Eigen::MatrixXf> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Vector3f eigenvalues = svd.singularValues();

    result.pca.pca1 = svd.matrixV().col(0);
    result.pca.pca2 = svd.matrixV().col(1);
    result.pca.pca3 = svd.matrixV().col(2);

    result.pca.eigenvalue1 = eigenvalues[0];
    result.pca.eigenvalue2 = eigenvalues[1];
    result.pca.eigenvalue3 = eigenvalues[2];

    result.pca.t1 = ((result.pca.pca1 * result.pca.eigenvalue1) / std::sqrt(nrPoints - 1)).norm();
    result.pca.t2 = ((result.pca.pca2 * result.pca.eigenvalue2) / std::sqrt(nrPoints - 1)).norm();

    result.valid = true;
    return result;
}


Vector3f SkeletonVertexAnalyzer::createMidVector(const Vector3f &vec1, const Vector3f &vec2)
{
    Vector3f d1 = vec1;
    Vector3f d2 = vec2;
    d1.normalize();
    d2.normalize();


    Vector3f mid((d1[0]+d2[0])/2.f, (d1[1]+d2[1])/2.f, (d1[2]+d2[2])/2.f);
    mid.normalize();
    return mid;
}


Diameter SkeletonVertexAnalyzer::calculateDiameter(Eigen::Vector3f &pos, std::vector<Eigen::Vector3f> &points)
{
    Diameter diameter;
    diameter.maxDiameter = 0.f;
    diameter.minDiameter = std::numeric_limits<float>::max();
    diameter.averageDiameter = 0.f;


//    std::cout << "pos: " << pos.transpose() << std::endl;
//    std::cout << "points: " << points.size() << std::endl;

    float e1;
    float e2;
    float e3;


    for (size_t i = 0; i < points.size(); i++)
    {
        Eigen::Vector3f p = points.at(i);
        e1 = p[0] - pos[0];
        e1 *= e1;

        e2 = p[1] - pos[1];
        e2 *= e2;

        e3 = p[2] - pos[2];
        e3 *= e3;


        float dist = std::sqrt(e1 + e2 + e3);

        if (dist > diameter.maxDiameter)
        {
            diameter.maxDiameter = dist;
        }

        if (dist < diameter.minDiameter)
        {
            diameter.minDiameter = dist;
        }

        diameter.averageDiameter += dist;

    }

    diameter.averageDiameter /= points.size();

    return diameter;
}

Diameter SkeletonVertexAnalyzer::getPlanesWithMeshPoints(const SimoxCGAL::SkeletonPtr skeleton, const SimoxCGAL::SurfaceMeshPtr mesh, const std::vector<SimoxCGAL::SkeletonVertex> &interval, const VirtualRobot::MathTools::Plane &splane, std::vector<Vector3f> &storePoints)
{
    std::vector<Diameter> diameters;

    Diameter result;
    result.averageDiameter = 0.f;
    result.maxDiameter = 0.f;
    result.minDiameter = 0.f;

    if (!skeleton)
    {
        return result;
    }

    for (size_t i = 0; i < interval.size(); i++)
    {
        SkeletonVertex sv = interval.at(i);

        Point pos = (*skeleton)[sv].point;
        Eigen::Vector3f posE(pos[0], pos[1], pos[2]);

        vector<Eigen::Vector3f> points;
        for (SurfaceMeshVertexDescriptor mvd : (*skeleton)[sv].vertices)
        {
            Point mvdp = get(CGAL::vertex_point, *mesh, mvd);
            Eigen::Vector3f mvdpe(mvdp[0], mvdp[1], mvdp[2]);
            Eigen::Vector3f pPlane = VirtualRobot::MathTools::projectPointToPlane(mvdpe, splane);
            points.push_back(pPlane);
            storePoints.push_back(pPlane);

        }

        if (points.size() != 0)
        {
            Diameter s = SkeletonVertexAnalyzer::calculateDiameter(posE, points);
//            s.print();
            diameters.push_back(s);
            points.clear();
        }
    }

//    std::cout << "diameters: " << diameters.size() << std::endl;

    if (diameters.size() == 0)
        return result;

    for (size_t i = 0; i < diameters.size(); i++)
    {
        result.averageDiameter += diameters.at(i).averageDiameter;
        result.maxDiameter += diameters.at(i).maxDiameter;
        result.minDiameter += diameters.at(i).minDiameter;

    }

    result.minDiameter /= diameters.size();
    result.maxDiameter /= diameters.size();
    result.averageDiameter /= diameters.size();

//    result.print();
    return result;
}


Eigen::Vector3f SkeletonVertexAnalyzer::pointToVector(Point point)
{
    return Eigen::Vector3f(point[0], point[1], point[2]);
}

}
