#include "SegmentedObjectIO.h"

#include <VirtualRobot/XML/BaseIO.h>

#include "Segmentation/Skeleton/SkeletonPart.h"

using namespace VirtualRobot;
using namespace std;

namespace SimoxCGAL {


SegmentedObjectPtr SegmentedObjectIO::Load(const std::string &file)
{
    // load file
    std::ifstream in(file.c_str());
    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << file);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string objectXML(buffer.str());
    in.close();


    char* y = new char[objectXML.size() + 1];
    strncpy(y, objectXML.c_str(), objectXML.size() + 1);

    rapidxml::xml_document<char> doc;    // character type defaults to char
    doc.parse<0>(y);    // 0 means default parse flags
    rapidxml::xml_node<char>* objectXMLFile = doc.first_node("SimoxCGAL-SegmentedObject");

    SegmentedObjectPtr segObject = SegmentedObjectPtr(new SegmentedObject);

    //segmentationTime?
    for (rapidxml::xml_node<>* child = objectXMLFile->first_node("SkeletonPart"); child != NULL; child = child->next_sibling("SkeletonPart"))
    {
        segObject->addObjectPart(SkeletonPart::fromXML(child));
    }

    return segObject;

}

bool SegmentedObjectIO::Save(SegmentedObjectPtr o, const std::string &filename)
{
    if (!o)
        return false;

    std::string xml = o->toXML(0);
    bool res = BaseIO::writeXMLFile(filename, xml, true);
    return res;
}

}
