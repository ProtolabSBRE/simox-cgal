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


    std::string toXML(int nrTabs = 1)
    {

        std::string t;
        std::string ta = "\t";
        for (int i=0;i<nrTabs;i++)
            t += "\t";
        std::stringstream ss;

        ss << t << "<SkeletonPoint vertex='" << vertex << "'>\n";
        ss << t << ta << "<Endpoint value='" << endpoint << "'/>\n";
        ss << t << ta << "<Branch value='" << branch << "'/>\n";
        ss << t << ta << "<Neighbor>\n";

        std::list<SimoxCGAL::SkeletonVertex>::iterator it;
        for (it = neighbor.begin(); it != neighbor.end(); it++)
        {
            SimoxCGAL::SkeletonVertex n = *it;
            ss << t << ta << ta << "<Vertex value='" << n << "'/>\n";
        }

        ss << t << ta << "</Neighbor>\n";
        ss << t << "</SkeletonPoint>\n";

        return ss.str();
    }
};

typedef boost::shared_ptr<SkeletonPoint> SkeletonPointPtr;

}

#endif // SKELETONPOINT_H
