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

SoSeparator* SkeletonVisualization::createSegmentationVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, vector<ObjectPartPtr> members, bool show_lines)
{
    SoSeparator* visu = new SoSeparator();
    visu->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visu->addChild(u);

    for (int i = 0; i < members.size(); i++)
    {
        SubpartPtr subpart = boost::static_pointer_cast<Subpart>(members.at(i));

        SoSeparator* segment = new SoSeparator();
        SoMaterial* color = new SoMaterial();

        if (subpart->palpable)
        {
            color->diffuseColor.setValue(1.f, 0.55f, 0.f);

        } else {

            // alle Objekte die zu klein sind (also keine Unterteilung in Intervalle möglich -> nicht greifbar)
            // und nicht greifbare Objekte
            color->diffuseColor.setValue(0.f, 0.f, 1.f);

        }

        segment->addChild(color);

        SoSeparator* s = createSegmentVisualization(skeleton, mesh, subpart, show_lines);
        segment->addChild(s);

        visu->addChild(segment);

    }

    visu->unrefNoDelete();
    return visu;
}



SoSeparator* SkeletonVisualization::createSegmentVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SubpartPtr subpart, bool show_lines)
{

    SoSeparator* visu = new SoSeparator;
    visu->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visu->addChild(u);

    std::vector<Eigen::Vector3f> lines;

    std::map<SkeletonVertex, SkeletonPointPtr>::iterator pair;
    for (pair = subpart->skeletonPart.begin(); pair != subpart->skeletonPart.end(); pair++)
    {

        SkeletonPointPtr p = pair->second;
        Eigen::Vector3f center((*skeleton)[p->vertex].point[0], (*skeleton)[p->vertex].point[1], (*skeleton)[p->vertex].point[2]);
        SoSeparator* s = VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(center, 0.3f, 0.f, 1.f, 0.f, 1.f);
        visu->addChild(s);

        std::list<SkeletonVertex>::iterator it;
        for (it = p->neighbor.begin(); it != p->neighbor.end(); it++)
        {
            Eigen::Vector3f point((*skeleton)[*it].point[0], (*skeleton)[*it].point[1], (*skeleton)[*it].point[2]);
            lines.push_back(point);
        }

        if (show_lines)
        {
            SoIndexedLineSet* p_lineSet = createConnectionVisualization(p->vertex, skeleton, mesh);
            visu->addChild(p_lineSet);
        }

        SoIndexedLineSet* t = createPolylinesVisualization(center, lines);
        visu->addChild(t);
        lines.clear();
    }


    visu->unrefNoDelete();
    return visu;

}

SoSeparator* SkeletonVisualization::createPigmentedMeshVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, vector<ObjectPartPtr> members, int part)
{
    SoSeparator* visu = new SoSeparator;
    visu->ref();

    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
//    visu->addChild(u);


    if (part >= members.size())
    {
        //alle Segmente einfärben

        for (int i = 0; i < members.size(); i++)
        {
            SubpartPtr subpart = boost::static_pointer_cast<Subpart>(members.at(i));

            SoSeparator* segment = new SoSeparator();
            VirtualRobot::VisualizationFactory::Color color;

            if (subpart->palpable)
            {
                color.r = 1.f;
                color.g = 0.45f;
                color.b = 0.f;

            } else {

                // alle Objekte die zu klein sind (also keine Unterteilung in Intervalle möglich -> nicht greifbar)
                // und nicht greifbare Objekte
                color.r = 0.f;
                color.g = 0.f;
                color.b = 1.f;

            }

            SoNode* s = createPigmentedSubpartVisualization(skeleton, mesh, subpart, color);
            segment->addChild(s);

            visu->addChild(segment);

        }

    } else {

        // "part" Segment einfärben
        SubpartPtr subpart = boost::static_pointer_cast<Subpart>(members.at(part));
        SoNode* s = createPigmentedSubpartVisualization(skeleton, mesh, subpart, VirtualRobot::VisualizationFactory::Color(1.f, 0.f, 0.f));
        visu->addChild(s);

    }

    return visu;
}

SoNode* SkeletonVisualization::createPigmentedSubpartVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SubpartPtr subpart, VirtualRobot::VisualizationFactory::Color color)
{
    map<SurfaceMeshVertexDescriptor, int> surfaceIndices;
    map<SurfaceMeshFaceDescriptor, vector<int>> faceVertices;

    VirtualRobot::TriMeshModelPtr triMesh(new VirtualRobot::TriMeshModel());

    int j = 0;

    for (auto& vertex : subpart->skeletonPart)
    {
        SkeletonVertex v = vertex.first;

        BOOST_FOREACH(SurfaceMeshVertexDescriptor vd, (*skeleton)[v].vertices) {
            Point a = get(CGAL::vertex_point, *mesh, vd);
            triMesh->addVertex(Eigen::Vector3f(a[0], a[1], a[2]));
            surfaceIndices[vd] = j;
            j++;
        }
    }


     map<SurfaceMeshVertexDescriptor, int> tmp = surfaceIndices;

    for (auto& vertex : tmp)
    {
        SurfaceMeshVertexDescriptor v = vertex.first;

        BOOST_FOREACH(SurfaceMeshFaceDescriptor f, CGAL::faces_around_target(mesh->halfedge(v), *mesh))
        {

            if (faceVertices.count(f) > 0)
            {
                //Face schon im Segment!
                //nicht mehr prüfen
                continue;
            }

            vector<int> index;
            BOOST_FOREACH(SurfaceMeshHalfedgeDescriptor hd, CGAL::halfedges_around_face(mesh->halfedge(f), *mesh))
            {

                if (surfaceIndices.count(mesh->target(hd)) > 0)
                {
                    index.push_back(surfaceIndices[mesh->target(hd)]);
                    continue;

                }

                if (surfaceIndices.count(mesh->target(hd)) == 0 && !subpart->palpable)
                {
                    //bei nicht greifbaren Segmenten -> Face auffüllen!
                    Point a = get(CGAL::vertex_point, *mesh, mesh->target(hd));
                    triMesh->addVertex(Eigen::Vector3f(a[0], a[1], a[2]));
                    surfaceIndices[mesh->target(hd)] = j;
                    j++;
                    index.push_back(surfaceIndices[mesh->target(hd)]);
                    continue;
                }

                index.push_back(-1);

            }


            if (index.size() != 3)
            {
                VR_ERROR << "No Trianlge in mesh!" << endl;
            }


            if (index.at(0) >= 0 && index.at(1) >= 0 && index.at(2) >= 0)
            {
                faceVertices[f] = index;
                VirtualRobot::MathTools::TriangleFace face;
                face.set(index.at(0), index.at(1), index.at(2));
                triMesh->addFace(face);

            }
        }
    }

    return VirtualRobot::CoinVisualizationFactory::getCoinVisualization(triMesh, false, color, false);
}

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
    SoVertexProperty* vp = new SoVertexProperty;

    SbVec3f pt(center[0], center[1], center[2]);
    vp->vertex.set1Value(0, pt);

    int j = 0;
    int k = 1;

    for (int i = 0; i < lines.size(); i++) {
        Eigen::Vector3f p = lines.at(i);

        SbVec3f pt1(p[0], p[1], p[2]);
        inLineSet->coordIndex.set1Value(3*j, 0);
        inLineSet->coordIndex.set1Value(3*j + 1, k);
        inLineSet->coordIndex.set1Value(3*j + 2, -1);
        vp->vertex.set1Value(k, pt1);
        j++;
        k++;

    }

    inLineSet->vertexProperty.setValue(vp);
    return inLineSet;
}
