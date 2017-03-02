#include "SkeletonPolyhedron.h"

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
    VR_INFO << "Calculation started of " << name << " : ...\n";

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
    skeleton = SkeletonPtr(new Skeleton(ske));

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    skeletonTimeMS = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count(); //milliseconds

    VR_INFO << "Calculation of "<< name << " done with " << num_vertices(*skeleton) << " vertices.\n";
    VR_INFO << "Time: " << skeletonTimeMS << " ms\n";
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

//SoSeparator* SkeletonPolyhedron::calculateSegmentSkeleton(Triangle_mesh& mesh, int number, int width, bool show_lines)
//{
//    SoSeparator* segment = new SoSeparator();
//    SoSeparator* skel_segment = new SoSeparator();
//    SoSeparator* show_linesSep = new SoSeparator();
//    vector<Eigen::Vector3f> skeleton_lines;

//    segment->ref();
//    SoUnits* u = new SoUnits();
//    u->units = SoUnits::MILLIMETERS;
//    segment->addChild(u);

//    SoMaterial* color_skeleton = new SoMaterial();
//    SoMaterial* color_lines = new SoMaterial();

//    color_skeleton->diffuseColor.setValue(1.f, 0.f, 0.f);
//    color_lines->diffuseColor.setValue(0.f, 1.f, 0.f);

//    skel_segment->addChild(color_skeleton);
//    show_linesSep->addChild(color_lines);

//    vector<Skeleton_vertex> segment_vertex;
//    Skeleton_vertex vertex = boost::vertex(number, skeleton);
//    Point a = skeleton[vertex].point;
//    Eigen::Vector3f aa(a[0], a[1], a[2]);
//    skel_segment->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(aa, 1.f, 0.f, 1.f, 1.f, 0.f));

//    rekursion(segment_vertex, vertex, vertex, width);

//    vector<Eigen::Vector3f> tri_lines;

//    for (int i = 0; i < segment_vertex.size(); i++)
//    {
//        Skeleton_vertex id = segment_vertex.at(i);
//        Eigen::Vector3f center(skeleton[id].point[0], skeleton[id].point[1], skeleton[id].point[2]);

//        if(show_lines)
//        {
//            SoIndexedLineSet* p = SkeletonVisualization::createConnectionVisualization(id, skeleton, mesh);
//            show_linesSep->addChild(p);
//        }

//        tri_lines.clear();

//        Skeleton_adjacency ai, ai_end;

//        for (boost::tie(ai, ai_end) = boost::adjacent_vertices(id, skeleton); ai != ai_end; ++ai)
//        {
//            Eigen::Vector3f point(skeleton[*ai].point[0], skeleton[*ai].point[1], skeleton[*ai].point[2]);
//            skeleton_lines.push_back(point);
//        }


//        SoIndexedLineSet* lineSet = SkeletonVisualization::createPolylinesVisualization(center, skeleton_lines);
//        skel_segment->addChild(lineSet);
//        skeleton_lines.clear();
//    }

//    segment->unrefNoDelete();
//    segment->addChild(show_linesSep);
//    segment->addChild(skel_segment);
//    return segment;

//}

//void SkeletonPolyhedron::rekursion(vector<Skeleton_vertex>& segment_global, Skeleton_vertex vertex_center, Skeleton_vertex vertex_not, int depth)
//{
//    vector<Skeleton_vertex> segment_local;

//    Skeleton_adjacency ai, ai_end;

//    for (boost::tie(ai, ai_end) = boost::adjacent_vertices(vertex_center, skeleton); ai != ai_end; ++ai) {
//        if (*ai != vertex_not)
//        {
//            double tmp = std::sqrt(CGAL::squared_distance(skeleton[vertex_center].point, skeleton[*ai].point));
//            distances.push_back(tmp);
//            segment_local.push_back(*ai);
//        }

//    }


//    if (depth != 1)
//    {
//        for (int i = 0; i < segment_local.size(); i++)
//        {
//            rekursion(segment_global, segment_local.at(i), vertex_center, depth - 1);
//        }
//    }

//    segment_global.push_back(vertex_center);

//}

SkeletonPtr SkeletonPolyhedron::getSkeleton()
{
    return skeleton;
}
