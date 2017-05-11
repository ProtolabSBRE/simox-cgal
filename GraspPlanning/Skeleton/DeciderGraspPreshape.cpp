#include "DeciderGraspPreshape.h"

namespace SimoxCGAL
{

DeciderGraspPreshape::DeciderGraspPreshape()
{
    minThicknessPower = 20.f;
    maxThicknessPower = 60.f;
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
    //float minLengthPower = 30.f;
    //float maxLengthPower = 150.f;

    if (thickness >= minThicknessPower && thickness <= maxThicknessPower /*&& length >= minLengthPower && length <= maxLengthPower*/)
    {
        return true;
    }

    return false;
}

bool DeciderGraspPreshape::decidePrecisionPreshape(float length, float thickness)
{
    //float minLengthPrecision = 1.0f;
    //float maxLengthPrecision = 30.0f;


    if (thickness > minThicknessPrecision && thickness < maxThicknessPrecision /*&& length > minLengthPrecision && length < maxLengthPrecision*/)
    {
        return true;
    }

    return false;
}
}
