#include "CGALSkeleton.h"

#include "Inventor/nodes/SoIndexedLineSet.h"
#include "Inventor/nodes/SoMaterial.h"


#include "Visualization/CoinVisualization/CGALCoinVisualization.h"


using namespace std;
using namespace VirtualRobot;

namespace SimoxCGAL {


CGALSkeleton::CGALSkeleton(const string &name, SurfaceMeshPtr mesh)
{
    this->name = name;
    this->mesh = mesh;
}

CGALSkeleton::CGALSkeleton(const string &name, SurfaceMeshPtr mesh, SkeletonPtr skeleton)
{
    this->name = name;
    this->mesh = mesh;
    this->skeleton = skeleton;
}

CGALSkeleton::~CGALSkeleton()
{

}

void CGALSkeleton::initParameters(int a1, double a2, double a3, double a4, int a5, bool a6, double a7)
{
    max_triangle_angle = a1 * (CGAL_PI / 180.0);
    quality_speed_tradeoff = a2;
    medially_centered_speed_tradeoff = a3;
    area_variation_factor = a4;
    max_iterations = a5;
    is_medially_centered = a6;
    min_edge_length = a7;
}

void CGALSkeleton::calculateSkeleton()
{
    Skeletonization mcs(*mesh);

    //Parameter einstellen für Skelettberechnung.
    mcs.set_max_triangle_angle(max_triangle_angle);
    mcs.set_quality_speed_tradeoff(quality_speed_tradeoff);
    mcs.set_medially_centered_speed_tradeoff(medially_centered_speed_tradeoff);
    mcs.set_area_variation_factor(area_variation_factor);
    mcs.set_max_iterations(max_iterations);
    mcs.set_is_medially_centered(is_medially_centered);
//    mcs.set_min_edge_length(min_edge_length);

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    mcs.contract_until_convergence();
    Skeleton ske;
    mcs.convert_to_skeleton(ske);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    skeleton = SkeletonPtr(new Skeleton(ske));

    skeletonTimeMS = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count(); //milliseconds
}

void CGALSkeleton::clear()
{
    skeleton->clear();
}

string CGALSkeleton::toXML()
{
    string t = "\t";
    string tt = t + "\t";
    string ttt = tt + "\t";
    stringstream ss;
    std::map<Point, int> vertices_map;

    ss << "<CGAL-Skeleton>\n";
    ss << t << "<NumberOfVertices vertices='" << num_vertices(*skeleton) <<"'/>\n";
    ss << t << "<NumberOfEdges edges='" << num_edges(*skeleton) << "'/>\n";
    ss << t << "<Parameters>\n";
    ss << t << t << "<MaxTriangleAngle value='" << max_triangle_angle << "'/>\n";
    ss << t << t << "<QualitySpeedTradeOff value='" << quality_speed_tradeoff << "'/>\n";
    ss << t << t << "<MediallyCenteredSpeedTradeOff value='" << medially_centered_speed_tradeoff << "'/>\n";
    ss << t << t << "<AreaVariationFactor value='" << area_variation_factor << "'/>\n";
    ss << t << t << "<MaxIterations value='" << max_iterations << "'/>\n";
    ss << t << t << "<IsMediallyCentered bool='" << is_medially_centered << "'/>\n";
    ss << t << t << "<MinEdgeLength value='" << min_edge_length << "'/>\n";
    ss << t << "</Parameters>\n";
    ss << t << "<Vertices>\n";

    int i = 0;

    //typedef boost::uint32_t size_type;

    BOOST_FOREACH(SkeletonVertex v, vertices(*skeleton))
    {
        ss << tt << "<Point index='" << i << "'>\n";
        ss << ttt << "<Coordinate x='" << (*skeleton)[v].point[0] << "' ";
        ss << "y='" << (*skeleton)[v].point[1] << "' ";
        ss << "z='" << (*skeleton)[v].point[2] << "'\n";
        ss << "/>\n";
        vertices_map[(*skeleton)[v].point] = i;
        i++;

        ss << ttt << "<IndexToMesh>\n";

        BOOST_FOREACH(SurfaceMeshVertexDescriptor vd, (*skeleton)[v].vertices)
        {
            ss << "<Index index='" << vd.operator size_type() << "'/>\n";
        }

        ss << ttt << "</IndexToMesh>\n";

        ss << "</Point>\n";
    }

    ss  << t << "</Vertices>\n";

    ss << t << "<Edges>\n";


    //wird so genutzt wie ich Liste bekomme!
    BOOST_FOREACH(SkeletonEdge e, edges(*skeleton))
    {
        const Point& src = (*skeleton)[source(e, *skeleton)].point;
        const Point& trg = (*skeleton)[target(e, *skeleton)].point;
        ss << tt << "<Edge from='" << vertices_map.at(src) << "' to='" << vertices_map.at(trg) << "'/>\n";
    }

    ss << t << "</Edges>\n";


    ss << "</CGAL-Skeleton>\n";
    return ss.str();
}


SkeletonPtr CGALSkeleton::getSkeleton()
{
    return skeleton;
}

int CGALSkeleton::getTime()
{
    return skeletonTimeMS;
}

}
