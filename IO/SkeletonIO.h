#ifndef SKELETON_H
#define SKELETON_H

#include <sstream>


#include "SimoxCGAL.h"
#include "CGALSkeleton.h"


//// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
}
namespace SimoxCGAL {

class SkeletonIO
{
public:

    static SimoxCGAL::SkeletonPtr loadSkeleton(const std::string& file);
    static SimoxCGAL::SkeletonPtr createSkeletonObject(rapidxml::xml_node<char> *skeletonNode);
    static bool saveSkeletonObject(SimoxCGAL::CGALSkeletonPtr s, const std::string& filename);

protected:
    //instance not allowed
    SkeletonIO();
    virtual ~SkeletonIO();
};

}
#endif // SKELETON_H
