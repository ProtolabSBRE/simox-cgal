#ifndef SKELETONPOINT_H
#define SKELETONPOINT_H


#include "SimoxCGAL.h"

namespace SimoxCGAL
{

struct SIMOX_CGAL_IMPORT_EXPORT SkeletonPoint
{
public:
    SimoxCGAL::SkeletonVertex vertex;
    bool endpoint;
    bool branch;
    std::list<SimoxCGAL::SkeletonVertex> neighbor;


    std::string toXML()
    {
        std::string t = "\t";
        std::stringstream ss;
        ss << "<SkeletonPoint vertex='" << vertex << "'>\n";
        ss << t << "<Endpoint value='" << endpoint << "'/>\n";
        ss << t << "<Branch value='" << branch << "'/>\n";
        ss << t << "<Neighbor>\n";

        std::list<SimoxCGAL::SkeletonVertex>::iterator it;
        for (it = neighbor.begin(); it != neighbor.end(); it++)
        {
            SimoxCGAL::SkeletonVertex n = *it;
            ss << t << t << "<Vertex value='" << n << "'/>\n";
        }

        ss << t << "</Neighbor>\n";
        ss << "</SkeletonPoint>\n";

        return ss.str();
    }
};

typedef boost::shared_ptr<SkeletonPoint> SkeletonPointPtr;

}

#endif // SKELETONPOINT_H
