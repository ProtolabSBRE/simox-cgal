#include "SkeletonPart.h"

#include <utility>
#include <limits>


#include <Inventor/nodes/SoSeparator.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>

using namespace std;

namespace SimoxCGAL {

SkeletonPart::SkeletonPart()
{

}

SkeletonPart::~SkeletonPart()
{

}

string SkeletonPart::toXML(int nrTabs)
{

    std::string t;
    std::string ta = "\t";
    for (int i=0;i<nrTabs;i++)
        t += "\t";
    std::stringstream ss;

    ss << t << "<SkeletonPart name='" << name << "'>\n";
    ss << t << ta << "<SegmentNumber number='" << segmentNumber << "'/>\n";
    ss << t << ta << "<Palpable bool='" << palpable << "'/>\n";
    ss << t << ta << "<NumberOfSkeletonPoints number='" << skeletonPart.size() << "'/>\n";
    ss << t << ta << "<LengthOfSegment length='" << lengthOfSegment << "'/>\n";


    ss << t  << ta << "<SortedSkeletonSegmentIndex>\n";

    for (int i = 0; i < sortedSkeletonPartIndex.size(); i++)
    {
        ss << t << ta << ta << "<Vertex value='" << sortedSkeletonPartIndex.at(i) << "'/>\n";
    }

    ss << t << ta << "</SortedSkeletonSegmentIndex>\n";

    ss << t << ta << "<SkeletonSegment>\n";

    std::map<SkeletonVertex, SkeletonPointPtr>::iterator p;
    for (p = skeletonPart.begin(); p != skeletonPart.end(); p++)
    {
        ss << p->second->toXML(nrTabs + 2);
    }

    ss << t << ta << "</SkeletonSegment>\n";

    ss << t << "</SkeletonPart>\n";
    return ss.str();
}


void SkeletonPart::calculateLengthOfSegment(SkeletonPtr skeleton)
{
    if (skeletonPart.size() <= 1)
    {
        lengthOfSegment = 0.0;
        return;
    }

    std::vector<Edge> edges;
    std::vector<double> length;


    std::map<SkeletonVertex, SkeletonPointPtr>::iterator pair;
    std::list<SkeletonVertex>::iterator vd;
    for (pair = skeletonPart.begin(); pair != skeletonPart.end(); pair++)
    {
        SkeletonPointPtr point = pair->second;

        for (vd = point->neighbor.begin(); vd != point->neighbor.end(); vd++)
        {
            SkeletonVertex v = *vd;
            Edge edge1(point->vertex, v);
            Edge edge2(v, point->vertex);
            bool found1 = std::find(edges.begin(), edges.end(), edge1) != edges.end();
            bool found2 = std::find(edges.begin(), edges.end(), edge2) != edges.end();
            double tmp = std::sqrt(CGAL::squared_distance((*skeleton)[point->vertex].point, (*skeleton)[v].point));

            if (!found1 && !found2)
            {
                edges.push_back(edge1);
                length.push_back(tmp);
            }
        }

    }

    lengthOfSegment = 0.0;

    for (int i = 0; i < length.size(); i++)
    {
        lengthOfSegment += length.at(i);
    }

    std::cout << "length of segment "<< segmentNumber << ": " << lengthOfSegment << std::endl;
}

bool SkeletonPart::calculateInterval(SkeletonPtr skeleton, int position, float length, Interval &storeInterval)
{

    if (skeletonPart.size() <= 1 || length > lengthOfSegment)
    {
        palpable = false;
        std::stringstream out;
        out << "NG";
        out << segmentNumber;
        name = out.str();
        return false;
    }

    if (!palpable)
    {
        return false;
    }

    SkeletonPointPtr sk_point = skeletonPart[sortedSkeletonPartIndex.at(position)];

    if (sk_point->endpoint)
    {
        std::cout << "Endpoint not supported.\n";
        return false;
    }

    if (sk_point->neighbor.size() != 2)
    {
        std::cout << "Segmentende!" << std::endl;
        return false;
    }

    SkeletonVertex n1 = sk_point->neighbor.front();
    SkeletonVertex n2 = sk_point->neighbor.back();

    storeInterval.push_back(sk_point->vertex);
    float oneSize = length / 2.f;

    bool validRight = false;
    bool validLeft = false;
    validRight = fillInterval(skeleton, n1, sk_point->vertex, storeInterval, oneSize);
    validLeft = fillInterval(skeleton, n2, sk_point->vertex, storeInterval, oneSize);

    if (!validRight || !validLeft)
    {
        std::cout << "Intervall not valid.\n";
        return false;
    }

    return true;
}

bool SkeletonPart::fillInterval(SkeletonPtr skeleton, SkeletonVertex &center, SkeletonVertex &not_vertex, Interval &interval, float& length)
{

    SkeletonPointPtr point = skeletonPart[center];

    SkeletonVertex nextNeighbor;
    int test = 0;


    //eig immer nur neighbor == 2 -> vlt if-else?
    std::list<SkeletonVertex>::iterator vd;
    for (vd = point->neighbor.begin(); vd != point->neighbor.end(); vd++)
    {
        if ((*vd) != not_vertex)
        {
            nextNeighbor = *vd;
            test++;
        }
    }

    if (test > 1)
    {
        std::cout << "SHOULD NOT BE THE CASE: nextNeighbor is " << test << " times set." << std::endl;
    }

    if (test == 0)
    {
        //endpunkte vom Segment!
        return false;
    }


    double tmp = std::sqrt(CGAL::squared_distance((*skeleton)[point->vertex].point, (*skeleton)[nextNeighbor].point));

    if ((float)tmp > length)
    {
        interval.push_back(nextNeighbor);
        return true;

    } else {

        float diff = length - (float)tmp;
        interval.push_back(center);
        return fillInterval(skeleton, nextNeighbor, center, interval, diff);
    }

}

}
