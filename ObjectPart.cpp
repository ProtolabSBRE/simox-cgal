#include "ObjectPart.h"

using namespace VirtualRobot;

namespace SimoxCGAL {

ObjectPart::ObjectPart()
{

}

ObjectPart::~ObjectPart()
{

}


void ObjectPart::addParameterDouble(const std::string &key, double value)
{
    parametersDouble[key] = value;
}

bool ObjectPart::hasParamterDouble(const std::string &key)
{
    return parametersDouble.find(key) != parametersDouble.end();
}

bool ObjectPart::getParameterDouble(const std::string &key, double &storeValue)
{
    if (!hasParamterDouble(key))
        return false;

    storeValue = parametersDouble[key];
    return true;
}


void ObjectPart::addParameterString(const std::string &key, const std::string &value)
{
    parametersString[key] = value;
}

bool ObjectPart::hasParamterString(const std::string &key)
{
    return parametersString.find(key) != parametersString.end();
}

bool ObjectPart::getParameterString(const std::string &key, std::string &storeValue)
{
    if (!hasParamterString(key))
        return false;

    storeValue = parametersString[key];
    return true;
}



}
