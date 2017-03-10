#include "CGALCoinVisualization.h"

#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoFaceSet.h>

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSeparator.h>

#include <Segmentation/Skeleton/SkeletonPart.h>

using namespace std;
using namespace VirtualRobot;

namespace SimoxCGAL
{

SoSeparator* CGALCoinVisualization::CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::PhongMaterial mat, VisualizationFactory::Color colorLine, float lineSize)
{
    SoSeparator* visu = new SoSeparator;

    if (points.size() == 0)
    {
        return visu;
    }

    visu->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visu->addChild(u);

    SoMaterial* m = new SoMaterial;
    m->diffuseColor.setValue(mat.diffuse.r, mat.diffuse.g, mat.diffuse.b);
    m->ambientColor.setValue(mat.ambient.r, mat.ambient.g, mat.ambient.b);
    m->emissiveColor.setValue(mat.emission.r, mat.emission.g, mat.emission.b);
    //        m->shininess.setValue(mat.shininess, mat.shininess, mat.shininess);
    m->specularColor.setValue(mat.specular.r, mat.specular.g, mat.specular.b);
    m->transparency.setValue(mat.transparency);
    visu->addChild(m);

    SoCoordinate3* coordinate3 = new SoCoordinate3;
    SoCoordinate3* coordinate3b = new SoCoordinate3;

    for (size_t i = 0; i < points.size(); i++)
    {
        SbVec3f pt(points[i](0), points[i](1), points[i](2));
        coordinate3->point.set1Value(i, pt);
        coordinate3b->point.set1Value(i, pt);
    }

    SbVec3f pt0(points[0](0), points[0](1), points[0](2));
    coordinate3b->point.set1Value(points.size(), pt0);
    visu->addChild(coordinate3);
    SoFaceSet* faceSet = new SoFaceSet;
    faceSet->numVertices.set1Value(0, points.size());
    visu->addChild(faceSet);

    // create line around polygon
    if (lineSize > 0.f && !colorLine.isNone())
    {
        SoSeparator* lineSep = new SoSeparator;
        visu->addChild(lineSep);
        SoMaterial* m2 = new SoMaterial;
        m2->diffuseColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        m2->ambientColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        m2->transparency.setValue(colorLine.transparency);
        lineSep->addChild(m2);
        lineSep->addChild(coordinate3b);

        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(lineSize);
        lineSep->addChild(lineSolutionStyle);

        SoLineSet* lineSet = new SoLineSet;
        lineSet->numVertices.set1Value(0, points.size() + 1);
        //faceSet->startIndex.setValue(0);
        lineSep->addChild(lineSet);
    }

    visu->unrefNoDelete();
    return visu;
}
SoNode *CGALCoinVisualization::CreateCoinVisualization(CGALPolyhedronMeshPtr mesh, bool showLines, bool showNormals)
{
    boost::associative_property_map<PolyhedronFacetDoubleMap> m;
    boost::associative_property_map<PolyhedronFacetIntMap> m2;
    return CreateCoinVisualization(mesh, m, 0, m2, 0, showLines, showNormals);
}

SoNode *CGALCoinVisualization::CreateCoinVisualizationSegments(CGALPolyhedronMeshPtr mesh,  boost::associative_property_map<PolyhedronFacetIntMap> &segment_property_map, size_t number_of_segments, bool showLines, bool showNormals)
{
    boost::associative_property_map<PolyhedronFacetDoubleMap> m;
    return CreateCoinVisualization(mesh, m, 0, segment_property_map, number_of_segments, showLines, showNormals);
}

SoNode* CGALCoinVisualization::CreateCoinVisualizationSDF(CGALPolyhedronMeshPtr mesh,  boost::associative_property_map<PolyhedronFacetDoubleMap>& sdf_raw_property_map, float maxSDF, bool showLines, bool showNormals)
{
    boost::associative_property_map<PolyhedronFacetIntMap> m2;
    return CreateCoinVisualization(mesh, sdf_raw_property_map, maxSDF, m2, 0, showLines, showNormals);
}

SoNode* CGALCoinVisualization::CreateCoinVisualization(CGALPolyhedronMeshPtr mesh,
                                                        boost::associative_property_map<PolyhedronFacetDoubleMap>& sdf_raw_property_map,
                                                       float maxSDF,
                                                        boost::associative_property_map<PolyhedronFacetIntMap>& segment_property_map,
                                                       size_t number_of_segments,
                                                       bool showLines, bool showNormals)
{
    SoSeparator* res = new SoSeparator;
    res->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    res->addChild(u);
    PolyhedronMeshPtr model = mesh->getMesh();

    float lineSize = 1.0f;
    VisualizationFactory::Color lineColor = VisualizationFactory::Color::Black();
    if (!showLines)
    {
        lineColor = VisualizationFactory::Color::None();
        lineSize = 0.0f;
    }
    VirtualRobot::ColorMap color = VirtualRobot::ColorMap::eHot;

    Eigen::Vector3f z(0, 0, 1.0f);
    SoSeparator* arrow = CoinVisualizationFactory::CreateArrow(z, 20.0f, 1.0f);
    arrow->ref();

    for (PolyhedronMesh::Facet_iterator it = model->facets_begin(); it != model->facets_end(); ++it)
    {
        PolyHalfedgeHandle halfedge, hInit;
        hInit = it->halfedge();         //Zugriff auf model(kein const)
        halfedge = it->halfedge();      //Zugriff auf model (kein const)

        std::vector<Eigen::Vector3f> v;
        Eigen::Vector3f v1;
        do
        {
            PointPoly p1 = halfedge->prev()->vertex()->point();
            v1 = Eigen::Vector3f(p1.x(), p1.y(), p1.z());
            v.push_back(v1);
            halfedge = halfedge->next();
        }
        while (hInit != halfedge);


        VisualizationFactory::Color colorInner;
        VisualizationFactory::PhongMaterial mat;
        mat.diffuse = VisualizationFactory::Color(0, 0, 0);
        mat.transparency = 0.0f;
        if (maxSDF>0)
        {
            if (color.getColor(sdf_raw_property_map[it] / (float)maxSDF, colorInner))
            {
                mat.ambient = colorInner;
                mat.diffuse = colorInner;
                mat.transparency = colorInner.transparency;
            }
            else
            {
                mat.ambient = VisualizationFactory::Color::Black();
            }
        }
        else if (number_of_segments>0)
        {
            //Determine color of segments
            if (color.getColor(segment_property_map[it] / (float)number_of_segments, colorInner))
            {
                mat.ambient = colorInner;
                mat.diffuse = colorInner;
                mat.transparency = colorInner.transparency;
            }
            else
            {
                mat.ambient = VisualizationFactory::Color::Black();
            }
        }
        else
        {
            mat.ambient = VisualizationFactory::Color::Red(0.0f);
        }
        SoSeparator* s;
        s = CreatePolygonVisualization(v, mat, lineColor, lineSize);
        res->addChild(s);



        if (showNormals)
        {
            Eigen::Vector3f v2 = Eigen::Vector3f::Zero();
            for (std::vector<Eigen::Vector3f>::iterator i = v.begin(); i != v.end(); ++i)
            {
                v2 += (*i);
            }
            if (v.size() == 0)
            {
                VR_ERROR << " Divided by Zero" << endl;
            } else
                v2 /= float(v.size());

            SoMatrixTransform* mt = new SoMatrixTransform;
            KernelPolyhedron::Vector_3 pNormal = CGAL::cross_product(hInit->next()->vertex()->point() - hInit->vertex()->point(),
                                       hInit->next()->next()->vertex()->point() - hInit->next()->vertex()->point());


            Eigen::Vector3f normal = Eigen::Vector3f(pNormal.x(), pNormal.y(), pNormal.z());
            MathTools::Quaternion q = MathTools::getRotation(z, normal);
            Eigen::Matrix4f mat = MathTools::quat2eigen4f(q);
            //Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
            mat.block(0, 3, 3, 1) = v2;
            SbMatrix m(reinterpret_cast<SbMat*>(mat.data()));
            mt->matrix.setValue(m);
            SoSeparator* sn = new SoSeparator();
            sn->addChild(mt);
            sn->addChild(arrow);
            res->addChild(sn);
        }

        v.erase(v.begin(), v.end());
    }

    arrow->unref();
    res->unrefNoDelete();
    return res;

}

SoSeparator* CGALCoinVisualization::CreateSkeletonVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, bool showLines)
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
            SoIndexedLineSet* p_lineSet = CreateConnectionVisualization(v, skeleton, mesh);
            visu->addChild(p_lineSet);
        }

        SoIndexedLineSet* lineSet = CreatePolylinesVisualization(center, lines);
        visu->addChild(lineSet);
        lines.clear();

    }

    visu->unrefNoDelete();
    return visu;
}

SoSeparator* CGALCoinVisualization::CreateSegmentationVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, vector<ObjectPartPtr> members, bool show_lines)
{
    SoSeparator* visu = new SoSeparator();
    visu->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visu->addChild(u);

    for (int i = 0; i < members.size(); i++)
    {
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(i));

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

        SoSeparator* s = CreateSegmentVisualization(skeleton, mesh, subpart, show_lines);
        segment->addChild(s);

        visu->addChild(segment);

    }

    visu->unrefNoDelete();
    return visu;
}



SoSeparator* CGALCoinVisualization::CreateSegmentVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SkeletonPartPtr subpart, bool show_lines)
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
            SoIndexedLineSet* p_lineSet = CreateConnectionVisualization(p->vertex, skeleton, mesh);
            visu->addChild(p_lineSet);
        }

        SoIndexedLineSet* t = CreatePolylinesVisualization(center, lines);
        visu->addChild(t);
        lines.clear();
    }


    visu->unrefNoDelete();
    return visu;

}

SoSeparator* CGALCoinVisualization::CreatePigmentedMeshVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, vector<ObjectPartPtr> members, int part)
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
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(i));

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

            SoNode* s = CreatePigmentedSubpartVisualization(skeleton, mesh, subpart, color);
            segment->addChild(s);

            visu->addChild(segment);

        }

    } else {

        // "part" Segment einfärben
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(part));
        SoNode* s = CreatePigmentedSubpartVisualization(skeleton, mesh, subpart, VirtualRobot::VisualizationFactory::Color(1.f, 0.f, 0.f));
        visu->addChild(s);

    }

    return visu;
}

SoNode* CGALCoinVisualization::CreatePigmentedSubpartVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SkeletonPartPtr subpart, VirtualRobot::VisualizationFactory::Color color)
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

SoIndexedLineSet* CGALCoinVisualization::CreateConnectionVisualization(SkeletonVertex& vertex, SkeletonPtr skeleton, SurfaceMeshPtr mesh)
{
    vector<Eigen::Vector3f> lines;
    Eigen::Vector3f center((*skeleton)[vertex].point[0], (*skeleton)[vertex].point[1], (*skeleton)[vertex].point[2]);

    BOOST_FOREACH(SurfaceMeshVertexDescriptor vd, (*skeleton)[vertex].vertices) {
       Point a = get(CGAL::vertex_point, *mesh, vd);

       Eigen::Vector3f point(a[0], a[1], a[2]);
       lines.push_back(point);
    }

    return CreatePolylinesVisualization(center, lines);
}

SoIndexedLineSet* CGALCoinVisualization::CreatePolylinesVisualization(Eigen::Vector3f center, vector<Eigen::Vector3f> lines)
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

SoSeparator* CGALCoinVisualization::ShowSkeletonPoint(SkeletonPtr skeleton, SurfaceMeshPtr mesh, int pointPosition)
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

    if (pointPosition >= num_vertices(*skeleton) || pointPosition < 0)
    {
        VR_INFO << "Point position doesn't exist in skeleton graph. Please select another position." << endl;
        VR_INFO << "Range are from 0 to " << (num_vertices(*skeleton) - 1) << endl;
        return s;
    }


    SkeletonVertex vertex = boost::vertex(pointPosition, *skeleton);
    Point a = (*skeleton)[vertex].point;
    Eigen::Vector3f aa(a[0], a[1], a[2]);
    s->addChild(VirtualRobot::CoinVisualizationFactory::CreateVertexVisualization(aa, 1.f, 0.f, 1.f, 0.f, 0.f));

    SoSeparator* l = new SoSeparator;
    SoMaterial* color = new SoMaterial;
    color->diffuseColor.setValue(0.f, 1.f, 0.f);
    l->addChild(color);
    SoIndexedLineSet* lines = CreateConnectionVisualization(vertex, skeleton, mesh);
    l->addChild(lines);
    s->addChild(l);

    return s;
}

}
