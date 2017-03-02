#include "SkeletonVisualization.h"

#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoMaterial.h>

#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

//#include "Polyhedron/Segmentation/SkeletonPoint.h"

using namespace std;
using namespace SimoxCGAL;

SoSeparator* SkeletonVisualization::createSkeletonVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, bool showLines)
{
    SoSeparator* visu = new SoSeparator;
    visu->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visu->addChild(u);


    vector<Eigen::Vector3f> lines;


    BOOST_FOREACH(SkeletonVertex v, vertices(*skeleton)) {

        Eigen::Vector3f center((*skeleton)[v].point[0], (*skeleton)[v].point[1], (*skeleton)[v].point[2]);

        SoSeparator* s = VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(center, 0.2f, 0.f, 1.f, 0.f, 1.f);
        visu->addChild(s);

        SkeletonAdjacency ai, ai_end;
        for (boost::tie(ai, ai_end) = boost::adjacent_vertices(v, *skeleton); ai != ai_end; ++ai) {

            Eigen::Vector3f point((*skeleton)[*ai].point[0], (*skeleton)[*ai].point[1], (*skeleton)[*ai].point[2]);
            lines.push_back(point);

        }

        if (showLines)
        {
            SoIndexedLineSet* p_lineSet = createConnectionVisualization(v, skeleton, mesh);
            visu->addChild(p_lineSet);
        }

        SoIndexedLineSet* lineSet = createPolylinesVisualization(center, lines);
        visu->addChild(lineSet);
        lines.clear();

    }

    visu->unrefNoDelete();
    return visu;
}

//SoSeparator* SkeletonVisualization::createSegmentationVisualization(Skeleton& skeleton, Triangle_mesh& mesh, vector<SubpartPtr> &members, bool show_lines)
//{
//    SoSeparator* visu = new SoSeparator();
//    visu->ref();
//    SoUnits* u = new SoUnits();
//    u->units = SoUnits::MILLIMETERS;
//    visu->addChild(u);

//    for (int i = 0; i < members.size(); i++)
//    {
//        SubpartPtr subpart = members.at(i);

//        SoSeparator* segment = new SoSeparator();
//        SoMaterial* color = new SoMaterial();

//        if (subpart->palpable)
//        {
//            color->diffuseColor.setValue(0.f, 1.f, 0.f);

//        } else {

//            // alle Objekte die zu klein sind (also keine Unterteilung in Intervalle möglich -> nicht greifbar)
//            // und nicht greifbare Objekte
//            color->diffuseColor.setValue(0.f, 0.f, 1.f);

//        }

//        segment->addChild(color);

//        SoSeparator* s = createSegmentVisualization(skeleton, mesh, subpart, show_lines);
//        segment->addChild(s);

//        visu->addChild(segment);

//    }

//    visu->unrefNoDelete();
//    return visu;
//}

//SoSeparator* SkeletonVisualization::createSegmentVisualization(Skeleton& skeleton, Triangle_mesh& mesh, SubpartPtr& subpart, bool show_lines)
//{
//    SoSeparator* visu = new SoSeparator;

//    visu->ref();
//    SoUnits* u = new SoUnits();
//    u->units = SoUnits::MILLIMETERS;
//    visu->addChild(u);

//    std::vector<Eigen::Vector3f> lines;

//    std::map<Skeleton_vertex, SkeletonPointPtr>::iterator pair;
//    for (pair = subpart->skeletonPart.begin(); pair != subpart->skeletonPart.end(); pair++)
//    {

//        SkeletonPointPtr p = pair->second;
//        Eigen::Vector3f center(skeleton[p->vertex].point[0], skeleton[p->vertex].point[1], skeleton[p->vertex].point[2]);
//        SoSeparator* s = VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(center, 0.3f, 0.f, 1.f, 0.f, 1.f);
//        visu->addChild(s);

//        std::list<Skeleton_vertex>::iterator it;
//        for (it = p->neighbor.begin(); it != p->neighbor.end(); it++)
//        {
//            Eigen::Vector3f point(skeleton[*it].point[0], skeleton[*it].point[1], skeleton[*it].point[2]);
//            lines.push_back(point);
//        }

//        if (show_lines)
//        {
//            SoIndexedLineSet* p_lineSet = createConnectionVisualization(p->vertex, skeleton, mesh);
//            visu->addChild(p_lineSet);
//        }

//        SoIndexedLineSet* t = createPolylinesVisualization(center, lines);
//        visu->addChild(t);
//        lines.clear();
//    }


//    visu->unrefNoDelete();
//    return visu;
//}

SoIndexedLineSet* SkeletonVisualization::createConnectionVisualization(SkeletonVertex& vertex, SkeletonPtr skeleton, SurfaceMeshPtr mesh)
{
    vector<Eigen::Vector3f> lines;
    Eigen::Vector3f center((*skeleton)[vertex].point[0], (*skeleton)[vertex].point[1], (*skeleton)[vertex].point[2]);

    BOOST_FOREACH(SurfaceMeshVertexDescriptor vd, (*skeleton)[vertex].vertices) {
       Point a = get(CGAL::vertex_point, *mesh, vd);

       Eigen::Vector3f point(a[0], a[1], a[2]);
       lines.push_back(point);
    }

    return createPolylinesVisualization(center, lines);
}

SoIndexedLineSet* SkeletonVisualization::createPolylinesVisualization(Eigen::Vector3f center, vector<Eigen::Vector3f> lines)
{

    SoIndexedLineSet* inLineSet = new SoIndexedLineSet;
    SoCoordinate3* coordinate3 = new SoCoordinate3;


    SbVec3f pt(center[0], center[1], center[2]);
    coordinate3->point.set1Value(0, pt);

    int j = 0;
    int k = 1;

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector3f p = lines.at(i);

        SbVec3f pt1(p[0], p[1], p[2]);
        inLineSet->coordIndex.set1Value(3*j, 0);
        inLineSet->coordIndex.set1Value(3*j + 1, k);
        inLineSet->coordIndex.set1Value(3*j + 2, -1);
        coordinate3->point.set1Value(k, pt1);
        j++;
        k++;

    }

    inLineSet->vertexProperty.setValue(coordinate3);
    return inLineSet;
}
