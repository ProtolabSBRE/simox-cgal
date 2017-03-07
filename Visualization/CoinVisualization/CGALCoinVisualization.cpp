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

SoNode *CGALCoinVisualization::CreateCoinVisualization(CGALPolyhedronMeshPtr mesh,
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


}
