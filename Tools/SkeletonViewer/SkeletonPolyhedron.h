#ifndef SKELETONPOLYHEDRON_H
#define SKELETONPOLYHEDRON_H

#include "SimoxCGAL.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/nodes/SoIndexedLineSet.h>


#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include <stdbool.h>
#include <iostream>
#include <algorithm>


using namespace std;
using namespace SimoxCGAL;

class SkeletonPolyhedron
{
public:
    SkeletonPolyhedron(const string &name, SurfaceMeshPtr mesh);

    SkeletonPolyhedron(const string &name, SurfaceMeshPtr mesh, SkeletonPtr skeleton);

    ~SkeletonPolyhedron();

    void initParameters(int a1 = 110,
                        double a2 = 0.1,
                        double a3 = 0.2,
                        double a4 = 0.0001,
                        int a5 = 500,
                        bool a6 = true,
                        double a7 = 0.0 /*currently not supported*/
                        );

    void calculateSkeleton();
    void clear();

    string toXML();

//    SoSeparator* calculateSegmentSkeleton(Triangle_mesh &mesh, int number, int width, bool show_lines);
//    void rekursion(vector<Skeleton_vertex>& segment_global, Skeleton_vertex vertex_center, Skeleton_vertex vertex_not, int depth);

    SkeletonPtr getSkeleton();



private:

    string name;

    SurfaceMeshPtr mesh;
    SkeletonPtr skeleton;
    //parameters
    double max_triangle_angle;
    double quality_speed_tradeoff;
    double medially_centered_speed_tradeoff;
    double area_variation_factor;
    int max_iterations;
    bool is_medially_centered;
    double min_edge_length;


    int skeletonTimeMS;

};

typedef boost::shared_ptr<SkeletonPolyhedron> SkeletonPolyhedronPtr;

#endif // SKELETTPOLYHEDRON_H
