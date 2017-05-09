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

    bool decidePowerPreshape(float length, float thickness);
    bool decidePrecisionPreshape(float length, float thickness);
};

typedef boost::shared_ptr<DeciderGraspPreshape> DeciderGraspPreshapePtr;

}

#endif // DECIDERGRASPPRESHAPE_H
