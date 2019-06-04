#include "DeciderGraspPreshape.h"

using namespace std;

namespace SimoxCGAL
{

DeciderGraspPreshape::DeciderGraspPreshape()
{
    minThicknessPower = 20.f;
    maxThicknessPower = 60.f;
    minLengthPower = 20.f;

    minThicknessPrecision = 1.f;
    maxThicknessPrecision = 20.f;
}

DeciderGraspPreshape::~DeciderGraspPreshape()
{

}


void DeciderGraspPreshape::setThicknessPrecision(float minThickness, float maxThickness)
{
    minThicknessPrecision = minThickness;
    maxThicknessPrecision = maxThickness;
}

void DeciderGraspPreshape::setThicknessPower(float minThickness, float maxThickness)
{
    minThicknessPower = minThickness;
    maxThicknessPower = maxThickness;
}

bool DeciderGraspPreshape::decidePowerPreshape(float length, float thickness)
{

    if ((thickness >= minThicknessPower && thickness <= maxThicknessPower) || length >= minLengthPower)
    {
        return true;
    }

    return false;
}

bool DeciderGraspPreshape::decidePrecisionPreshape(float /*length*/, float thickness)
{

    if (thickness > minThicknessPrecision && thickness < maxThicknessPrecision /*&& length > minLengthPrecision && length < maxLengthPrecision*/)
    {
        return true;
    }

    return false;
}
}
