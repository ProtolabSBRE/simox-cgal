#include "SkeletonPart.h"

#include <utility>
#include <limits>


#include <Inventor/nodes/SoSeparator.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/XML/BaseIO.h>

using namespace std;
using namespace VirtualRobot;

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

    for (size_t i = 0; i < sortedSkeletonPartIndex.size(); i++)
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

    std::vector<SkeletonIntervalEdge> edges;
    std::vector<double> length;

    std::list<SkeletonVertex>::iterator vd;

    for (size_t i = 0; i < sortedSkeletonPartIndex.size(); i++)
    {
        SkeletonVertex vertex = sortedSkeletonPartIndex.at(i);
        SkeletonPointPtr point = skeletonPart.at(vertex);

        for (vd = point->neighbor.begin(); vd != point->neighbor.end(); vd++)
        {
            SkeletonVertex v = *vd;
            SkeletonIntervalEdge edge1(point->vertex, v);
            SkeletonIntervalEdge edge2(v, point->vertex);
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

    for (size_t i = 0; i < length.size(); i++)
    {
        lengthOfSegment += length.at(i);
    }

    std::cout << "length of segment "<< segmentNumber << ": " << lengthOfSegment << std::endl;
}

bool SkeletonPart::calculateInterval(SkeletonPtr skeleton, int position, float length, bool endpoint, SkeletonInterval &storeInterval, bool verbose)
{



    if ((skeletonPart.size() <= 1 || length > lengthOfSegment) && !containsEndpoint())
    {
        palpable = false;
        std::stringstream out;
        out << "NG";
        out << segmentNumber;
        name = out.str();
        return false;
    }

    if (containsEndpoint())
    {
        palpable = true;
    }

    if (!palpable)
    {
        return false;
    }

    SkeletonPointPtr sk_point = skeletonPart[sortedSkeletonPartIndex.at(position)];

    float oneSize = length / 2.f;

    if (endpoint && !sk_point->endpoint)
    {
        VR_ERROR << "requesting endpoint interval, but skeleton point is no endpoint" << endl;
        return false;
    }

    if (endpoint)
    {
        SkeletonVertex n1 = sk_point->neighbor.front();
        storeInterval.push_back(sk_point->vertex);
        bool valid = fillIntervalEndpoint(skeleton, n1, sk_point->vertex, storeInterval, oneSize);

        if (!valid)
        {
            if (verbose)
                VR_INFO << "Endpoint not valid" << endl;
            return false;
        }

        return true;
    }

    if (sk_point->neighbor.size() != 2)
    {
        if (verbose)
            VR_INFO << "End of segment" << std::endl;
        return false;
    }

    SkeletonVertex n1 = sk_point->neighbor.front();
    SkeletonVertex n2 = sk_point->neighbor.back();

    storeInterval.push_back(sk_point->vertex);

    bool validRight = false;
    bool validLeft = false;
    validRight = fillInterval(skeleton, n1, sk_point->vertex, storeInterval, oneSize);
    validLeft = fillInterval(skeleton, n2, sk_point->vertex, storeInterval, oneSize);

    if (!validRight || !validLeft)
    {
        if (verbose)
            VR_INFO << "Interval not valid.\n";
        return false;
    }

    return true;
}

bool SkeletonPart::fillInterval(SkeletonPtr skeleton, SkeletonVertex &center, SkeletonVertex &not_vertex, SkeletonInterval &interval, float& length)
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
        // we need to add also the center
        interval.push_back(center);
        interval.push_back(nextNeighbor);
        return true;

    } else {

        float diff = length - (float)tmp;
        interval.push_back(center);
        return fillInterval(skeleton, nextNeighbor, center, interval, diff);
    }

}

bool SkeletonPart::fillIntervalEndpoint(SkeletonPtr skeleton, SkeletonVertex &center, SkeletonVertex &not_vertex, SkeletonInterval &interval, float& length)
{

    SkeletonPointPtr point = skeletonPart[center];

    SkeletonVertex nextNeighbor;
    int test = 0;

    cout << "begin" << endl;

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
        cout << "END" << endl;
        //endpunkte vom Segment!
        return true;
    }


    double tmp = std::sqrt(CGAL::squared_distance((*skeleton)[point->vertex].point, (*skeleton)[nextNeighbor].point));

    if ((float)tmp > length)
    {
        // we need to add also the center
        interval.push_back(center);
        interval.push_back(nextNeighbor);
        return true;

    } else {

        float diff = length - (float)tmp;
        interval.push_back(center);
        return fillIntervalEndpoint(skeleton, nextNeighbor, center, interval, diff);
    }

}

ObjectPartPtr SkeletonPart::fromXML(rapidxml::xml_node<> *node)
{
    SkeletonPartPtr subpart(new SkeletonPart);

    //name
    string name(BaseIO::processNameAttribute(node));
    subpart->name = name;


    //segmentNumber
    rapidxml::xml_node<>* segNumber = node->first_node("SegmentNumber", 0, false);
    rapidxml::xml_attribute<>* tmp_segNumber = segNumber->first_attribute();
    subpart->segmentNumber = BaseIO::convertToInt(tmp_segNumber->value());

    //palpable
    rapidxml::xml_node<>* p = node->first_node("Palpable", 0, false);
    rapidxml::xml_attribute<>* tmp_p = p->first_attribute();
    subpart->palpable = BaseIO::convertToInt(tmp_p->value()) != 0;

    //numberOfSkeletonPoints
    rapidxml::xml_node<>* num_points = node->first_node("NumberOfSkeletonPoints", 0, false);
    rapidxml::xml_attribute<>* tmp_num_points = num_points->first_attribute();
    int number_points = BaseIO::convertToInt(tmp_num_points->value());


    //lengthOfSegment
    rapidxml::xml_node<>* length = node->first_node("LengthOfSegment", 0, false);
    rapidxml::xml_attribute<>* tmp_length = length->first_attribute();
    subpart->lengthOfSegment = BaseIO::convertToFloat(tmp_length->value());

    //sortedSkeletonPart
    rapidxml::xml_node<>* sortedSkeleton = node->first_node("SortedSkeletonSegmentIndex", 0, false);
    std::vector<SkeletonVertex> sorted = loadSortedSegment(sortedSkeleton, subpart->palpable);
    subpart->sortedSkeletonPartIndex = sorted;


    //skeletonSegment
    rapidxml::xml_node<>* segments = node->first_node("SkeletonSegment", 0, false);

    std::map<SkeletonVertex,SkeletonPointPtr> points;

    for (rapidxml::xml_node<>* child = segments->first_node(); child != NULL; child = child->next_sibling())
    {

        SkeletonPointPtr point = loadSkeletonPoint(child);
        points[point->vertex] = point;
    }

    subpart->skeletonPart = points;

    THROW_VR_EXCEPTION_IF(size_t(number_points) != points.size(), "Wrong number of skeletonPoints");

    return subpart;

}

vector<SkeletonVertex> SkeletonPart::loadSortedSegment(rapidxml::xml_node<char> *node, bool palpaple)
{
    vector<SkeletonVertex> sorted;

    if (!palpaple)
    {
        return sorted;
    }

    for (rapidxml::xml_node<>* child = node->first_node(); child != NULL; child = child->next_sibling())
    {

        rapidxml::xml_attribute<>* vertex = child->first_attribute("value");
        SkeletonVertex v(BaseIO::convertToInt(vertex->value()));
        sorted.push_back(v);
    }

    return sorted;

}

SkeletonPointPtr SkeletonPart::loadSkeletonPoint(rapidxml::xml_node<char> *node)
{

    SkeletonPointPtr point(new SkeletonPoint);

    //vertex
    rapidxml::xml_attribute<>* vertex = node->first_attribute("vertex");
    SkeletonVertex v(BaseIO::convertToInt(vertex->value()));
    point->vertex = v;


    //endpoint
    rapidxml::xml_node<>* endpoint = node->first_node("Endpoint", 0, false);
    rapidxml::xml_attribute<>* tmp_endpoint = endpoint->first_attribute("value", 0, false);
    point->endpoint = BaseIO::convertToInt(tmp_endpoint->value()) != 0;


    //branch
    rapidxml::xml_node<>* branch = node->first_node("Branch", 0, false);
    rapidxml::xml_attribute<>* tmp_branch = branch->first_attribute("value", 0, false);
    point->branch = BaseIO::convertToInt(tmp_branch->value()) != 0;


    //neighbor
    rapidxml::xml_node<>* neighbor = node->first_node("Neighbor", 0, false);

    for(rapidxml::xml_node<>* child = neighbor->first_node(); child != NULL; child = child->next_sibling())
    {
        rapidxml::xml_attribute<>* n_vertex = child->first_attribute("value", 0, false);
        SkeletonVertex s_vertex(BaseIO::convertToInt(n_vertex->value()));
        point->neighbor.push_back(s_vertex);


    }

    return point;
}

bool SkeletonPart::containsEndpoint()
{
    for (auto item : skeletonPart)
    {
        if (item.second->endpoint)
            return true;
    }

    return false;
}



}
