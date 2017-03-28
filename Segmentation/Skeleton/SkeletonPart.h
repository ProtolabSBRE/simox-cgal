#ifndef SUBPART_H
#define SUBPART_H

#include "SimoxCGAL.h"

#include <string>

#include "VirtualRobot/Visualization/TriMeshModel.h"
#include "SkeletonPoint.h"
#include "ObjectPart.h"


namespace SimoxCGAL {

typedef std::vector<SimoxCGAL::SkeletonVertex>                              SkeletonInterval;
typedef std::pair<SimoxCGAL::SkeletonVertex, SimoxCGAL::SkeletonVertex>     SkeletonIntervalEdge;
//typedef std::map<SimoxCGAL::SkeletonVertex, SimoxCGAL::SkeletonPointPtr>::iterator SkeletonVertexIterator;

class SIMOX_CGAL_IMPORT_EXPORT SkeletonPart : public ObjectPart
{

public:

    SkeletonPart();
    ~SkeletonPart();

    std::string toXML(int nrTabs = 1);
    void calculateLengthOfSegment(SimoxCGAL::SkeletonPtr skeleton);
    bool calculateInterval(SimoxCGAL::SkeletonPtr skeleton, int position, float length, bool endpoint, SkeletonInterval &storeInterval, bool verbose = false);

    std::map<SimoxCGAL::SkeletonVertex,SkeletonPointPtr> skeletonPart;
    std::vector<SimoxCGAL::SkeletonVertex> sortedSkeletonPartIndex;
    std::vector<SkeletonInterval> intervalSet;
    std::vector<SimoxCGAL::SkeletonVertex> intervalCenter;
    std::string name;
    int segmentNumber;
    bool palpable;
    double lengthOfSegment;

    static ObjectPartPtr fromXML(rapidxml::xml_node<> *node);

protected:

    bool fillInterval(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SkeletonVertex &center, SimoxCGAL::SkeletonVertex &not_vertex, SkeletonInterval &interval, float& length);
    static std::vector<SkeletonVertex> loadSortedSegment(rapidxml::xml_node<char> *node, bool palpaple);
    static SkeletonPointPtr loadSkeletonPoint(rapidxml::xml_node<char> *node);

};

typedef boost::shared_ptr<SkeletonPart> SkeletonPartPtr;

}


#endif // SUBPART_H
