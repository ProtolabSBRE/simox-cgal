#ifndef PARTANALYSIS_H
#define PARTANALYSIS_H

#include <Inventor/nodes/SoSeparator.h>

#include <VirtualRobot/MathTools.h>

#include "SimoxCGAL.h"
#include "SegmentedObject.h"
#include "Math.h"
/*
#define POWER_GRASP "Power Preshape"
#define POWER_INTERVALL 94.f

#define PRECISION_GRASP "Precision Preshape"
#define PRECISION_INTERVALL 47.f
*/
/*
enum {
    ROUND,
    RECTANGULAR
};
*/
class PartAnalysis
{
public:

    PartAnalysis(SimoxCGAL::SegmentedObjectPtr segmentation, SimoxCGAL::SurfaceMeshPtr mesh);
    ~PartAnalysis();

    bool analyse(const int& part, const SimoxCGAL::SkeletonVertex &vertex);

    std::string getDecidedPreshape();
    PrincipalAxis3D getPCA();

    void setPrecisionMin(float precisionMin);
    float getPrecisionMin();

    void setPrecisionMax(float precisionMax);
    float getPrecisionMax();

    void setPowerMin(float powerMin);
    float getPowerMin();

    void setPowerMax(float powerMax);
    float getPowerMax();

protected:

    SimoxCGAL::SegmentedObjectPtr segmentation;
    SimoxCGAL::SurfaceMeshPtr mesh;

    PrincipalAxis3D pca;
    std::string decidedPreshape;
    //grasping plane
    VirtualRobot::MathTools::Plane plane;

    float precisionMin;
    float precisionMax;

    float powerMin;
    float powerMax;

    //visu
    SoSeparator* visu;


    bool analysePrecision();
    bool analysePower();

};

typedef boost::shared_ptr<PartAnalysis> PartAnalysisPtr;

#endif // PARTANALYSIS_H
