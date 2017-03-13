#ifndef DECIDERGRASPPRESHAPE_H
#define DECIDERGRASPPRESHAPE_H

#include <VirtualRobot/MathTools.h>
#include "Math.h"
#include "SimoxCGAL.h"

#include <string>
#include <vector>
#include <Eigen/Dense>

class DeciderGraspPreshape
{
public:
    DeciderGraspPreshape();
    ~DeciderGraspPreshape();

    bool decidePowerPreshape(float &thick);
    bool decidePrecisionPreshape(float &thick);
};

typedef boost::shared_ptr<DeciderGraspPreshape> DeciderGraspPreshapePtr;

#endif // DECIDERGRASPPRESHAPE_H
