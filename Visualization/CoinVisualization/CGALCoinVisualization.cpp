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
#include <Inventor/nodes/SoIndexedLineSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/SbColor.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>

#include <Segmentation/Skeleton/SkeletonPart.h>
#include <GraspPlanning/Skeleton/SkeletonVertexAnalyzer.h>

using namespace std;
using namespace VirtualRobot;
using namespace Eigen;

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

    SoMaterial* color = new SoMaterial();
    color->diffuseColor.setValue(1.f, 0.f, 0.f);

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
            SoSeparator* p_lineSet = CreateConnectionVisualization(v, skeleton, mesh);
            p_lineSet->addChild(color);
            visu->addChild(p_lineSet);
        }

        SoSeparator* lineSet = CreatePolylinesVisualization(center, lines);
        lineSet->addChild(color);
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

    for (size_t i = 0; i < members.size(); i++)
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
            SoSeparator* p_lineSet = CreateConnectionVisualization(p->vertex, skeleton, mesh);
            visu->addChild(p_lineSet);
        }

        SoSeparator* t = CreatePolylinesVisualization(center, lines);
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


    if ((size_t)part >= members.size())
    {
        //alle Segmente einfärben

        for (size_t i = 0; i < members.size(); i++)
        {
            SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(i));

            SoNode* s = CreatePigmentedSubpartVisualization(skeleton, mesh, subpart);
            visu->addChild(s);

        }

    } else {

        // "part" Segment einfärben
        SkeletonPartPtr subpart = boost::static_pointer_cast<SkeletonPart>(members.at(part));
        SoNode* s = CreatePigmentedSubpartVisualization(skeleton, mesh, subpart);
        visu->addChild(s);

    }

    return visu;
}

SoNode* CGALCoinVisualization::CreatePigmentedSubpartVisualization(SkeletonPtr skeleton, SurfaceMeshPtr mesh, SkeletonPartPtr subpart)
{

    SoSeparator* res = new SoSeparator;
    res->ref();

    map<SurfaceMeshVertexDescriptor, int> surfaceIndices;
    map<SurfaceMeshVertexDescriptor, SkeletonVertex> meshToSkeletonVertex;
    map<SurfaceMeshFaceDescriptor, int> faceColor;

    VirtualRobot::VisualizationFactory::Color color_segment;
    VirtualRobot::VisualizationFactory::Color color_endpoint;
    color_endpoint.r = 1.f;
    color_endpoint.g = 0.f;
    color_endpoint.b = 0.f;


    if (subpart->palpable)
    {
        color_segment.r = 1.f;
        color_segment.g = 0.45f;
        color_segment.b = 0.f;

    } else {

        // alle Objekte die zu klein sind (also keine Unterteilung in Intervalle möglich -> nicht greifbar)
        // und nicht greifbare Objekte
        color_segment = VirtualRobot::VisualizationFactory::Color::Blue();

    }

    int j = 0;

    for (auto& vertex : subpart->skeletonPart)
    {
        SkeletonVertex v = vertex.first;

        if (vertex.second->endpoint)
        {
            cout << "endpoint: " << v << endl;
        }

        BOOST_FOREACH(SurfaceMeshVertexDescriptor vd, (*skeleton)[v].vertices) {
            surfaceIndices[vd] = j;
            meshToSkeletonVertex[vd] = v;
            j++;
        }
    }


    for (auto vertex : surfaceIndices)
    {
        SurfaceMeshVertexDescriptor v = vertex.first;

        BOOST_FOREACH(SurfaceMeshFaceDescriptor f, CGAL::faces_around_target(mesh->halfedge(v), *mesh))
        {
            if (faceColor.count(f) > 0)
            {
                //Farbe für Face schon bestimmt!
                continue;
            }

            vector<SurfaceMeshVertexDescriptor> index;
            vector<Eigen::Vector3f> face_coord;

            BOOST_FOREACH(SurfaceMeshHalfedgeDescriptor hd, CGAL::halfedges_around_face(mesh->halfedge(f), *mesh))
            {
                index.push_back(mesh->target(hd));

                Point a = get(CGAL::vertex_point, *mesh, mesh->target(hd));
                face_coord.push_back(Eigen::Vector3f(a[0], a[1], a[2]));
            }

            if (index.size() != 3)
            {
                VR_ERROR << "No triangle in mesh!" << endl;
            }

            //prüfe welche Farbe bzw. ob es gemalt wird!
            if (meshToSkeletonVertex.count(index.at(0)) > 0 && meshToSkeletonVertex.count(index.at(1)) > 0 && meshToSkeletonVertex.count(index.at(2)) > 0)
            {

                if (meshToSkeletonVertex[index.at(0)] == meshToSkeletonVertex[index.at(1)] && meshToSkeletonVertex[index.at(1)] == meshToSkeletonVertex[index.at(2)])
                {
                    SkeletonVertex v_tmp = meshToSkeletonVertex[index.at(0)];

                    if (subpart->skeletonPart[v_tmp]->endpoint)
                    {

                        // endpoint_color
                        res->addChild(VirtualRobot::CoinVisualizationFactory::CreatePolygonVisualization(face_coord, color_endpoint, color_endpoint, 0.f));
                    } else {
                        res->addChild(VirtualRobot::CoinVisualizationFactory::CreatePolygonVisualization(face_coord, color_segment, color_segment, 0.f));
                        // segment_color
                    }
                } else {
                    //segment color
                     res->addChild(VirtualRobot::CoinVisualizationFactory::CreatePolygonVisualization(face_coord, color_segment, color_segment, 0.f));
                }

            } else {

                if (subpart->palpable)
                {
                    //keine Farbe
                    continue;
                } else {
                    //segment_farbe
                     res->addChild(VirtualRobot::CoinVisualizationFactory::CreatePolygonVisualization(face_coord, color_segment, color_segment, 0.f));
                }

            }

            faceColor[f] = 1;
        }
    }

    return res;
}

SoSeparator* CGALCoinVisualization::CreateConnectionVisualization(SkeletonVertex& vertex, SkeletonPtr skeleton, SurfaceMeshPtr mesh)
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

SoSeparator* CGALCoinVisualization::CreatePolylinesVisualization(Eigen::Vector3f center, vector<Eigen::Vector3f> lines)
{
    SoSeparator* res = new SoSeparator;
    res->ref();

    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    res->addChild(u);

    if (lines.size() == 0)
        return res;

    SoIndexedLineSet* inLineSet = new SoIndexedLineSet;
    SoVertexProperty* vertexProperty = new SoVertexProperty;

    SbVec3f* vertexPositions = new SbVec3f[1 + lines.size()];
    vertexPositions[0].setValue(center[0], center[1], center[2]);

    for (int i = 0; i < lines.size(); i++)
    {
        vertexPositions[i + 1].setValue(lines.at(i)[0], lines.at(i)[1], lines.at(i)[2]);
    }

    vertexProperty->vertex.setValues(0, 1 + lines.size(), vertexPositions);
    inLineSet->vertexProperty = vertexProperty;

    delete[] vertexPositions;


    int32_t* l = new int32_t[lines.size() * 3];

    for (int32_t i = 0; i < (int32_t)lines.size(); i++)
    {
        l[i * 3] = 0;
        l[i * 3 + 1] = i + 1;
        l[i * 3 + 2] = -1;

    }

    //add line vector
    inLineSet->coordIndex.setValues(0, lines.size() * 3, l);
    res->addChild(inLineSet);

    delete[] l;


//    res->unrefNoDelete();
    return res;
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
    SoSeparator* lines = CreateConnectionVisualization(vertex, skeleton, mesh);
    l->addChild(lines);
    s->addChild(l);

    return s;
}

SoSeparator* CGALCoinVisualization::CreateGraspVisualization(GraspPtr grasp, ManipulationObjectPtr object)
{
    SoSeparator* sep = new SoSeparator;
    sep->ref();
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    sep->addChild(u);

    Eigen::Matrix4f m = grasp->getTcpPoseGlobal(object->getGlobalPose());
    Eigen::Vector3f pos = m.block(0,3,3,1);

    Eigen::Vector3f x = m.block(0, 0, 3, 1);
    Eigen::Vector3f y = m.block(0, 1, 3, 1);
    Eigen::Vector3f z = m.block(0, 2, 3, 1);

    SoTransform* trans = new SoTransform;
    SbMatrix mt;
    SbVec3f vec(pos[0],pos[1], pos[2]);
    mt.setTranslate(vec);
    trans->setMatrix(mt);
    sep->addChild(trans);

    SbVec3f* vertexPositions = new SbVec3f[4];
    vertexPositions[0].setValue(x[0], x[1], x[2]);
    vertexPositions[1].setValue(y[0], y[1], y[2]);
    vertexPositions[2].setValue(z[0], z[1], z[2]);
    vertexPositions[3].setValue(0, 0, 0);

    SoCoordinate3* coord = new SoCoordinate3;
    coord->point.setValues(0, 4, vertexPositions);
    sep->addChild(coord);


    SoIndexedLineSet* set = new SoIndexedLineSet;

    SoMaterialBinding* binding = new SoMaterialBinding;
    binding->value = binding->PER_PART_INDEXED;
    sep->addChild(binding);

    SoMaterial* material = new SoMaterial;
    SoBaseColor* color = new SoBaseColor;

    SbColor* c = new SbColor[3];
    c[0].setValue(1.f, 0.f, 0.f);
    c[1].setValue(0.f, 1.f, 0.f);
    c[2].setValue(0.f, 0.f, 1.f);
    color->rgb.setValues(0, 3, c);
    sep->addChild(color);

    material->diffuseColor.setValues(0, 3, c);
    sep->addChild(material);



    int32_t* coordIndex = new int32_t[9];
    int32_t* colorIndex = new int32_t[3];

    coordIndex[0] = 0;
    coordIndex[1] = 3;
    coordIndex[2] = SO_END_LINE_INDEX;
    coordIndex[3] = 1;
    coordIndex[4] = 3;
    coordIndex[5] = SO_END_LINE_INDEX;
    coordIndex[6] = 2;
    coordIndex[7] = 3;
    coordIndex[8] = SO_END_LINE_INDEX;

    colorIndex[0] = (int32_t)0;
    colorIndex[1] = (int32_t)1;
    colorIndex[2] = (int32_t)2;

    set->coordIndex.setValues(0, 9, coordIndex);
    set->materialIndex.setValues(0, 3, colorIndex);
    sep->addChild(set);


    delete[] vertexPositions;
    delete[] c;
    delete[] coordIndex;
    delete[] colorIndex;

    return sep;
}

SoNode* CGALCoinVisualization::CreateGraspIntervalVisualization(SkeletonVertexResult result, SurfaceMeshPtr mesh, bool showPoints)
{
    SoSeparator* res = new SoSeparator;
    res->ref();

    SoSeparator* lines = new SoSeparator;

    SoMaterial* color = new SoMaterial;
    color->diffuseColor.setValue(1.f, 0.f, 0.f);
    lines->addChild(color);

    SkeletonVertex vertex;
    Point p;

    Eigen::Vector3f center;
    Eigen::Vector3f nb;

    vector<Eigen::Vector3f> points;
    int neighbors = 0;

    SkeletonPartPtr part = result.part;

    for (int i = 0; i < result.interval.size(); i++)
    {
        vertex = result.interval.at(i); //center
        p = (*result.skeleton)[vertex].point;
        center = Eigen::Vector3f(p[0], p[1], p[2]);

        for (SkeletonVertex n : part->skeletonPart[vertex]->neighbor)
        {
            if (find(result.interval.begin(), result.interval.end(), n) != result.interval.end())
            {
                p = (*result.skeleton)[n].point;
                nb = Eigen::Vector3f(p[0], p[1], p[2]);
                points.push_back(nb);
                neighbors++;
            }
        }

        //zeichne Linie vom Punkt!
        lines->addChild(CreatePolylinesVisualization(center, points));

        if (showPoints)
        {
            for(SurfaceMeshVertexDescriptor vd : (*result.skeleton)[vertex].vertices)
            {
                 Point a = get(CGAL::vertex_point, *mesh, vd);
                 res->addChild(CoinVisualizationFactory::CreateVertexVisualization(Vector3f(a[0], a[1], a[2]), 3.f, 0.f, 1.f, 0.f, 1.f));
            }
        }

        if (neighbors == 1 && !part->skeletonPart[vertex]->endpoint)
        {
            //interval end -> create plane!
            res->addChild(CoinVisualizationFactory::CreatePlaneVisualization(center, nb - center, 100.f, 0.f, false, 1.f, 0.f, 0.f));
        }

        points.clear();
        neighbors = 0;
    }

    p = (*result.skeleton)[result.skeletonPoint->vertex].point;
    center = Eigen::Vector3f(p[0], p[1], p[2]);

    res->addChild(CoinVisualizationFactory::CreateVertexVisualization(center, 5.f, 0.f, 0.f, 1.f, 0.f));
    res->addChild(CoinVisualizationFactory::CreatePlaneVisualization(result.graspingPlane.p, result.graspingPlane.n, 100.f, 0.f, false, 0.f, 1.f, 0.f));

    res->addChild(lines);

    return res;
}

SoNode* CGALCoinVisualization::CreateProjectedPointsVisualization(SkeletonVertexResult result, SurfaceMeshPtr mesh)
{
    SoSeparator* res = new SoSeparator;
    res->ref();

    vector<Vector3f> points;

    SkeletonVertexAnalyzer::getPlanesWithMeshPoints(result.skeleton, mesh, result.interval, result.graspingPlane, points);

    for (int i = 0; i < points.size(); i++)
        res->addChild(CoinVisualizationFactory::CreateVertexVisualization(points.at(i), 3.f, 0.f, 1.f, 0.f, 1.f));


     res->addChild(CoinVisualizationFactory::CreatePlaneVisualization(result.graspingPlane.p, result.graspingPlane.n, 200.f, 0.f, false, 0.f, 1.f, 0.f));

    SoSeparator* t = new SoSeparator;
    SoUnits* u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    t->addChild(u);

    SoTranslation* trans = new SoTranslation;
    trans->translation.setValue(result.graspingPlane.p[0], result.graspingPlane.p[1], result.graspingPlane.p[2]);
    t->addChild(trans);

    t->addChild(CoinVisualizationFactory::CreateArrow(result.pca.pca1, result.pca.t1, 3.f, VisualizationFactory::Color::Red()));
    t->addChild(CoinVisualizationFactory::CreateArrow(result.pca.pca2, result.pca.t2, 3.f, VisualizationFactory::Color::Green()));

    res->addChild(t);
    return res;

}

}
