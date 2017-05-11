#ifndef DECIDERGRASPPRESHAPE_H
#define DECIDERGRASPPRESHAPE_H

#include <VirtualRobot/MathTools.h>
#include "SkeletonVertexAnalyzer.h"
#include "SimoxCGAL.h"

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace SimoxCGAL
{

class DeciderGraspPreshape
{
public:
    DeciderGraspPreshape();
    ~DeciderGraspPreshape();

    void setThicknessPrecision(float minThickness, float maxThickness);
    void setThicknessPower(float minThickness, float maxThickness);

    bool decidePowerPreshape(float length, float thickness);
    bool decidePrecisionPreshape(float length, float thickness);
protected:
    float minThicknessPrecision;
    float minThicknessPower;
    float maxThicknessPrecision;
    float maxThicknessPower;
};

typedef boost::shared_ptr<DeciderGraspPreshape> DeciderGraspPreshapePtr;

}

#endif // DECIDERGRASPPRESHAPE_H
