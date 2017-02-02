#include "SegmentedObject.h"

using namespace VirtualRobot;

namespace SimoxCGAL {

SegmentedObject::SegmentedObject()
{

}

SegmentedObject::~SegmentedObject()
{

}


void SegmentedObject::addParameterDouble(const std::string &key, double value)
{
    parametersDouble[key] = value;
}

bool SegmentedObject::hasParamterDouble(const std::string &key)
{
    return parametersDouble.find(key) != parametersDouble.end();
}

bool SegmentedObject::getParameterDouble(const std::string &key, double &storeValue)
{
    if (!hasParamterDouble(key))
        return false;

    storeValue = parametersDouble[key];
    return true;
}


void SegmentedObject::addParameterString(const std::string &key, const std::string &value)
{
    parametersString[key] = value;
}

bool SegmentedObject::hasParamterString(const std::string &key)
{
    return parametersString.find(key) != parametersString.end();
}

bool SegmentedObject::getParameterString(const std::string &key, std::string &storeValue)
{
    if (!hasParamterString(key))
        return false;

    storeValue = parametersString[key];
    return true;
}



}
