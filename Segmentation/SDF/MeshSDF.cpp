#include "MeshSDF.h"
#include <CGAL/mesh_segmentation.h>

using namespace VirtualRobot;

namespace SimoxCGAL {

MeshSDF::MeshSDF(CGALPolyhedronMeshPtr mesh,
                 size_t paramNrRays,
                 double paramConeAngle,
                 size_t paramNrClusters)
    :mesh(mesh), paramNrRays(paramNrRays), paramConeAngle(paramConeAngle), paramNrClusters(paramNrClusters)
{
    VR_ASSERT(mesh);

    segmentationSDF();
}

MeshSDF::~MeshSDF()
{

}

bool MeshSDF::checkMeshValid()
{
    if (!mesh)
        return false;
    bool is_pure_triangle = mesh->getMesh()->is_pure_triangle();
    bool is_close = mesh->getMesh()->is_closed();
    bool is_valid = mesh->getMesh()->is_valid(false, 1);

    if (!is_pure_triangle)
    {
        VR_WARNING << "mesh is not pure triangle" << endl;
        //return false;
    }
    if (!is_close)
    {
        VR_WARNING << "mesh is not closed" << endl;
        //return false;
    }
    if (!is_valid)
    {
        VR_WARNING << "mesh is not valid" << endl;
        //return false;
    }

    return true;
}



bool MeshSDF::segmentationSDF()
{
    if (!checkMeshValid())
    {
        VR_ERROR << "mesh not valid..." << endl;
        return false;
    }

    internal_segment_map.clear();

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    // create a property-map
    //boost::associative_property_map<PolyhedronFacetDoubleMap> tmp(facetMap);
    //sdf_raw_property_map = tmp; //sdf_property_map: values of sdf

    // compute SDF values
    //this->min_max_sdf = CGAL::sdf_values(mesh, sdf_raw_property_map, paramConeAngle, paramNrRays, false);

    // create a property-map for SDF values
    PolyhedronFacetDoubleMap internal_seg_sdf_map;
    boost::associative_property_map<PolyhedronFacetDoubleMap> sdf_property_map(internal_seg_sdf_map);


    //calulate segmentation
    std::pair<double, double> min_max = CGAL::sdf_values(*(mesh->getMesh()), sdf_property_map, paramConeAngle, paramNrRays, true);

    // create a property-map for segment-ids

    boost::associative_property_map<PolyhedronFacetIntMap> tmp2(internal_segment_map);
    //boost::associative_property_map<PolyhedronFacetIntMap> tmp3(internal_cluster_map);
    this->segment_property_map = tmp2;
    //this->cluster_property_map = tmp3;
    // segment the mesh using default parameters for number of levels, and smoothing lambda
    // Any other scalar values can be used instead of using SDF values computed using the CGAL function
    //number_of_cluster = UI.spinBoxMaxCluster->value();
    number_of_segments = CGAL::segmentation_from_sdf_values(*(mesh->getMesh()), sdf_property_map, segment_property_map, paramNrClusters);//, 0.26, false);
    //number_of_cluster = CGAL::segmentation_from_sdf_values(cgalObject, sdf_property_map, cluster_property_map, number_of_cluster, 0.26, true);

    bool print = true;
    if (print)
    {
        std::cout << "Number of segments: " << number_of_segments << std::endl;
        //std::cout << "Number of clusters: " << number_of_cluster << std::endl;
        // print segment-ids
        std::cout << "['segment id']";
        for (PolyhedronMesh::Facet_const_iterator facet_it = mesh->getMesh()->facets_begin(); facet_it != mesh->getMesh()->facets_end(); ++facet_it)
        {
            // ids are between [0, number_of_segments -1]
            std::cout << "[";
            std::cout << segment_property_map[facet_it] << "]";
            //std::cout << cluster_property_map[facet_it] << "] ";

        }
        std::cout << std::endl;
        // print minimum & maximum SDF values
        std::cout << "minimum SDF: " << min_max_sdf.first << " maximum SDF: " << min_max_sdf.second << std::endl;
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    segmentationTimeMS = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count(); //milliseconds
    return true;

}

}
