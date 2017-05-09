#include "DeciderGraspPreshape.h"

namespace SimoxCGAL
{

DeciderGraspPreshape::DeciderGraspPreshape()
{

}

DeciderGraspPreshape::~DeciderGraspPreshape()
{

}

bool DeciderGraspPreshape::decidePowerPreshape(float length, float thickness)
{
    float minThickPower = 20.f;
    float maxThickPower = 60.f;
    float minLengthPower = 30.f;
    float maxLengthPower = 150.f;

    if (thickness >= minThickPower && thickness <= maxThickPower /*&& length >= minLengthPower && length <= maxLengthPower*/)
    {
        return true;
    }

    return false;
}

bool DeciderGraspPreshape::decidePrecisionPreshape(float length, float thickness)
{
    float minThickPrecision = 1.f;
    float maxThickPrecision = 20.f;
    float minLengthPrecision = 1.0f;
    float maxLengthPrecision = 30.0f;


    if (thickness > minThickPrecision && thickness < maxThickPrecision /*&& length > minLengthPrecision && length < maxLengthPrecision*/)
    {
        return true;
    }

    return false;
}
}
