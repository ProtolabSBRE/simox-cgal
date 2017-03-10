#include "SegmentedObject.h"

using namespace VirtualRobot;

namespace SimoxCGAL {

SegmentedObject::SegmentedObject()
{

}

SegmentedObject::~SegmentedObject()
{

}


void SegmentedObject::addObjectPart(ObjectPartPtr part)
{
    parts.push_back(part);
}

void SegmentedObject::addObjectParts(std::vector<ObjectPartPtr> parts)
{
    for (auto p:parts)
    {
        this->parts.push_back(p);
    }
}


std::vector<ObjectPartPtr> SegmentedObject::getObjectParts()
{
    return parts;
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

std::string SegmentedObject::toXML(int nrTabs)
{

    std::string t;
    std::string ta = "\t";
    for (int i=0;i<nrTabs;i++)
        t += "\t";
    std::stringstream ss;

    ss << t << "<SimoxCGAL-SegmentedObject>\n";

    std::string key = "method";
    std::string method;

    if (!getParameterString(key, method))
    {
        VR_ERROR << "No segmentation method found." << endl;
    }

    ss << t << ta << "<Method method='" << method << "'/>\n";


    for (int i = 0; i < parts.size(); i++)
    {
        ss << parts.at(i)->toXML(nrTabs + 1);
    }


    ss << t << "</SimoxCGAL-SegmentedObject>\n";

    return ss.str();
}



}
