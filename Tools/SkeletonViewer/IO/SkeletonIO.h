#ifndef SKELETON_H
#define SKELETON_H

#include <sstream>


#include "SimoxCGAL.h"


// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}

class SkeletonIO
{
public:

    static SimoxCGAL::Skeleton loadSkeleton(const std::string& file);
    static SimoxCGAL::Skeleton createSkeletonObject(rapidxml::xml_node<char> *skeletonNode);
    static std::string saveSkeletonObject(const std::string &name, const std::string& object, const std::string& basePath);

protected:
    //instance not allowed
    SkeletonIO();
    virtual ~SkeletonIO();
};

#endif // SKELETON_H
