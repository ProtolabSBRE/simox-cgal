#include "SegmentedObjectIO.h"

#include <VirtualRobot/XML/BaseIO.h>

using namespace VirtualRobot;

namespace SimoxCGAL {

bool SegmentedObjectIO::Save(SegmentedObjectPtr o, const std::string &filename)
{
    if (!o)
        return false;

    std::string xml = o->toXML(0);
    bool res = BaseIO::writeXMLFile(filename, xml, true);
    return res;
}

}
