#include "SkeletonPolyhedron.h"

#include "Inventor/nodes/SoIndexedLineSet.h"
#include "Inventor/nodes/SoMaterial.h"


#include "SkeletonVisualization.h"


using namespace std;
using namespace VirtualRobot;
using namespace SimoxCGAL;


SkeletonPolyhedron::SkeletonPolyhedron(const string &name, SurfaceMeshPtr mesh)
{
    this->name = name;
    this->mesh = mesh;
}

SkeletonPolyhedron::SkeletonPolyhedron(const string &name, SurfaceMeshPtr mesh, SkeletonPtr skeleton)
{
    this->name = name;
    this->mesh = mesh;
    this->skeleton = skeleton;
}

SkeletonPolyhedron::~SkeletonPolyhedron()
{

}

void SkeletonPolyhedron::initParameters(int a1, double a2, double a3, double a4, int a5, bool a6, double a7)
{
    max_triangle_angle = a1 * (CGAL_PI / 180.0);
    quality_speed_tradeoff = a2;
    medially_centered_speed_tradeoff = a3;
    area_variation_factor = a4;
    max_iterations = a5;
    is_medially_centered = a6;
    min_edge_length = a7;
}

void SkeletonPolyhedron::calculateSkeleton()
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

void SkeletonPolyhedron::clear()
{
    //name.clear();
    skeleton->clear();
//    mesh.clear();
    //distances.clear();
//    dist_to_faces.clear();
//    segmentid_of_faces.clear();
    //id_map.clear();
    //id_facets.clear();
    //number_of_segments = 0;
}

string SkeletonPolyhedron::toXML()
{
    string t = "\t";
    string tt = t + "\t";
    string ttt = tt + "\t";
    stringstream ss;
    std::map<Point, int> vertices_map;

    ss << "<Skeleton>\n";
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


    ss << "</Skeleton>\n";
    return ss.str();
}


SoSeparator* SkeletonPolyhedron::showPoint(int point)
{
    SoSeparator* s = new SoSeparator;
    s->ref();

    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    s->addChild(u);

    if (!skeleton)
    {
        return s;
    }

    if (point >= num_vertices(*skeleton))
    {
        return s;
    }


    SkeletonVertex vertex = boost::vertex(point, *skeleton);
    Point a = (*skeleton)[vertex].point;
    Eigen::Vector3f aa(a[0], a[1], a[2]);
    s->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(aa, 1.f, 0.f, 1.f, 0.f, 0.f));

    SoSeparator* l = new SoSeparator;
    SoMaterial* color = new SoMaterial;
    color->diffuseColor.setValue(0.f, 1.f, 0.f);
    l->addChild(color);
    SoIndexedLineSet* lines = SkeletonVisualization::createConnectionVisualization(vertex, skeleton, mesh);
    l->addChild(lines);

    s->addChild(l);

    return s;
}


SkeletonPtr SkeletonPolyhedron::getSkeleton()
{
    return skeleton;
}

int SkeletonPolyhedron::getTime()
{
    return skeletonTimeMS;
}
