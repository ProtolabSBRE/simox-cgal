#include "MeshSkeleton.h"

using namespace VirtualRobot;
using namespace std;

namespace SimoxCGAL {

MeshSkeleton::MeshSkeleton(CGALSurfaceMeshPtr mesh, SkeletonPtr skeleton, double width)
    : mesh(mesh), skeleton(skeleton), width(width), segmentedObject(new SegmentedObject)
{

    VR_ASSERT(mesh);
    VR_ASSERT(skeleton);

    segmentationSkeleton();
    string key = "method";
    string value = "skeleton";
    segmentedObject->addParameterString(key,value);

}

MeshSkeleton::~MeshSkeleton()
{

}

bool MeshSkeleton::segmentationSkeleton()
{
    id_map = std::vector<int>(num_vertices(*skeleton));

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    searchBranches();

//    std::cout << "\npointmap" << std::endl;
//    for (int i = 0; i < id_map.size(); i++)
//    {
//        std::cout << "[" << id_map.at(i) << "]";
//    }

//    std::cout << endl;


    //build Segment
    for(int i = 0; i< branchList.size(); i++)
    {
        SkeletonPartPtr subpart(new SkeletonPart);
        subpart->palpable = false;
        SkeletonVertex center = branchList.at(i);
        subpart->segmentNumber = id_map[center];

        std::stringstream out;
        out << "NG";
        out << subpart->segmentNumber;
        subpart->name = out.str();

        recursionBranchSegmentation(skeleton, center, width);

        for (int j = 0; j < pointMap.size(); j++)
        {

            if (id_map[pointMap.at(j)->vertex] == id_map[center])
            {
                SkeletonPointPtr point = pointMap.at(j);
                subpart->skeletonPart[point->vertex] = point;
            }
        }

        segmentedObject->addObjectPart(subpart);
    }

//    std::cout << "\npointmap" << std::endl;
//    for (int i = 0; i < id_map.size(); i++)
//    {
//        std::cout << "[" << id_map.at(i) << "]";
//    }

//    std::cout << endl;

    updateVertices();

    int id = segmentedObject->getObjectParts().size();

    std::list<SkeletonVertex>::iterator ita;
    for (ita = endpointList.begin(); ita != endpointList.end(); ita++)
    {

        SkeletonVertex vertex = *ita;

        if (id_map[vertex] >= 0 && id_map[vertex] != id)
        {
            //Vertex schon gesetzt! -> nächster Endpunkt
            continue;
        }

        SkeletonPartPtr segment(new SkeletonPart);
        segment->palpable = true;
        id_map[vertex] = id;

        std::stringstream out;
        out << "G";
        out << id;
        segment->name = out.str();

        segment->segmentNumber = id_map[vertex];
        recursionEndpointSegmentation(vertex, segment);

        segmentedObject->addObjectPart(segment);
        id++;
    }


//    std::cout << "\npointmap" << std::endl;
//    for (int i = 0; i < id_map.size(); i++)
//    {
//        std::cout << "[" << id_map.at(i) << "]";
//    }

//    cout << endl;
//    cout << "Segments: " << segmentedObject->getObjectParts().size() << endl;



    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    segmentationTimeMS = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count(); //milliseconds
    return true;
}

void MeshSkeleton::recursionBranchSegmentation(SkeletonPtr skeleton, SkeletonVertex center, float width)
{

    std::list<SkeletonVertex>::iterator it;
    for (it = pointMap[center]->neighbor.begin(); it != pointMap[center]->neighbor.end(); ++it)
    {

        SkeletonVertex neighbor = *it;

        float tmp = std::sqrt(CGAL::squared_distance((*skeleton)[center].point, (*skeleton)[neighbor].point));
        float result = width - tmp;
        bool found = std::find(endpointList.begin(), endpointList.end(), neighbor) != endpointList.end();

        if (id_map[neighbor] < 0 && result >= 0.f)
        {
            id_map[neighbor] = id_map[center];
            recursionBranchSegmentation(skeleton, neighbor, result);

        }


        if (!found && id_map[neighbor] < 0)
        {
            endpointList.push_back(neighbor);

        } else if (found){
            id_map[neighbor] = id_map[center];
            endpointList.remove(neighbor);
        }
    }
}

void MeshSkeleton::recursionEndpointSegmentation(SkeletonVertex center, SkeletonPartPtr subpart)
{

    if (pointMap[center]->neighbor.size() == 1)
    {
        std::cout << "START\n";
    }

    SkeletonVertex currentVertex = center;
    SkeletonVertex prevVertex = center;
    bool valid = true;
    int endpoints = 0;

    while (valid)
    {
        std::list<SkeletonVertex>::iterator v;
        std::list<SkeletonVertex> map = pointMap[currentVertex]->neighbor;
        for (v = map.begin(); v != map.end(); v++)
        {
            SkeletonVertex vd = *v;

//            if (pointMap[vd]->neighbor.size() != 2)
//            {
//                cout << "2->Vertex hat nicht 2 Nachbarn!-> solle nur bei Endpunkten vorkommen!\n";
//            }

            if (vd != prevVertex)
            {
                id_map[vd] = id_map[currentVertex];
                subpart->skeletonPart[currentVertex] = pointMap.at(currentVertex);
                subpart->sortedSkeletonPartIndex.push_back(currentVertex);
                prevVertex = currentVertex;
                currentVertex = vd;
                break;

            }

            if (pointMap[currentVertex]->neighbor.size() == 1)
            {
                endpoints++;

                if (endpoints > 1)
                {
                    cout << "ENDE" << endl;
                    id_map[vd] = id_map[currentVertex];
                    subpart->skeletonPart[currentVertex] = pointMap.at(currentVertex);
                    subpart->sortedSkeletonPartIndex.push_back(currentVertex);
                    valid = false;

                }
            }
        }
    }

}

bool MeshSkeleton::searchBranches()
{
    bool branch = false;
    int id = 0;
    BOOST_FOREACH(SkeletonVertex v , vertices(*skeleton))
    {
        Skeleton::degree_size_type out_number = boost::out_degree(v, *skeleton);
        SkeletonPointPtr point(new SkeletonPoint);
        point->vertex = v;

        if (out_number > 2)
        {
            cout << "Branch: " << v << endl;
            point->endpoint = false;
            point->branch = true;
            branchList.push_back(v);
            id_map[v] = id;
            branch = true;
            id++;

        } else if (out_number == 1)
        {
            cout << "Endpoint" << v << endl;
            point->endpoint = true;
            point->branch = false;
            endpointList.push_back(v);
            id_map[v] = -1;

        } else
        {
            point->branch = false;
            point->endpoint = false;
            id_map[v] = -1;
        }

        SkeletonAdjacency ai, ai_end;
        for (boost::tie(ai, ai_end) = boost::adjacent_vertices(v, *skeleton); ai != ai_end; ++ai)
        {
            point->neighbor.push_back(*ai);
        }

        pointMap.push_back(point);
    }

    return branch;

}

void MeshSkeleton::updateVertices()
{
    std::list<SkeletonVertex>::iterator it, it_n;
    for (it = endpointList.begin(); it != endpointList.end(); it++)
    {
        SkeletonVertex vertex = *it;

        for (it_n = pointMap.at(vertex)->neighbor.begin(); it_n != pointMap.at(vertex)->neighbor.end(); it_n++)
        {

            SkeletonVertex vertex2 = *it_n;

            if (id_map[vertex] != id_map[vertex2])
            {
                it_n = pointMap.at(vertex)->neighbor.erase(it_n);
                pointMap.at(vertex2)->neighbor.remove(vertex);
            }
        }

    }

}

SegmentedObjectPtr MeshSkeleton::getSegmentedObject()
{
    return segmentedObject;
}

int MeshSkeleton::getTime()
{
    return segmentationTimeMS;
}

}

