#include "DeciderGraspPreshape.h"

namespace SimoxCGAL
{

DeciderGraspPreshape::DeciderGraspPreshape()
{

}

DeciderGraspPreshape::~DeciderGraspPreshape()
{

}

bool DeciderGraspPreshape::decidePowerPreshape(float &thick)
{
    float minPrecision = 20.f;
    float maxPrecision = 60.f;

    if (minPrecision <= thick && thick <= maxPrecision)
    {
        return true;
    }

    return false;


}

bool DeciderGraspPreshape::decidePrecisionPreshape(float &thick)
{
    float minPrecision = 1.f;
    float maxPrecision = 20.f;

    if (minPrecision < thick && thick < maxPrecision)
    {
        return true;
    }

    return false;

}
}
