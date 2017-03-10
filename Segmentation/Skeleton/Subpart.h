#ifndef SUBPART_H
#define SUBPART_H

#include "SimoxCGAL.h"

#include <string>

#include "VirtualRobot/Visualization/TriMeshModel.h"
#include "SkeletonPoint.h"
#include "ObjectPart.h"


typedef std::vector<SimoxCGAL::SkeletonVertex>         Interval;
typedef std::pair<SimoxCGAL::SkeletonVertex, SimoxCGAL::SkeletonVertex>     Edge;
typedef std::map<SimoxCGAL::SkeletonVertex, SimoxCGAL::SkeletonPointPtr>::iterator SkeletonVertexIterator;

namespace SimoxCGAL {

class SIMOX_CGAL_IMPORT_EXPORT Subpart : public ObjectPart
{

public:

    std::string toXML();
    void calculateLengthOfSegment(SimoxCGAL::SkeletonPtr skeleton);
    bool calculateInterval(SimoxCGAL::SkeletonPtr skeleton, int position, float length, Interval &storeInterval);


    std::map<SimoxCGAL::SkeletonVertex,SkeletonPointPtr> skeletonPart;
    std::vector<SimoxCGAL::SkeletonVertex> sortedSkeletonPartIndex;
    std::vector<Interval> intervalSet;
    std::vector<SimoxCGAL::SkeletonVertex> intervalCenter;
    std::string name;
    int segmentNumber;
    bool palpable;
    double lengthOfSegment;

protected:



    bool fillInterval(SimoxCGAL::SkeletonPtr skeleton, SimoxCGAL::SkeletonVertex &center, SimoxCGAL::SkeletonVertex &not_vertex, Interval &interval, float& length);

};

typedef boost::shared_ptr<Subpart> SubpartPtr;

}


#endif // SUBPART_H
