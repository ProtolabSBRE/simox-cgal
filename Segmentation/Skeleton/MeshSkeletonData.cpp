#include "MeshSkeletonData.h"

#include <VirtualRobot/XML/rapidxml.hpp>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/BaseIO.h>

#include "IO/CGALMeshIO.h"
#include "IO/SkeletonIO.h"
#include "IO/SegmentedObjectIO.h"


using namespace std;
using namespace VirtualRobot;

namespace SimoxCGAL {


MeshSkeletonData::MeshSkeletonData()
{

}

MeshSkeletonData::~MeshSkeletonData()
{

}

MeshSkeletonDataPtr MeshSkeletonData::loadSkeletonData(const std::string &filename)
{

    VR_INFO << "Begin loading ... \n";


    std::ifstream in(filename.c_str());
    THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << filename);

    std::stringstream buffer;
    buffer << in.rdbuf();
    std::string objectXML(buffer.str());
    in.close();

    char* y = new char[objectXML.size() + 1];
    strncpy(y, objectXML.c_str(), objectXML.size() + 1);

    rapidxml::xml_document<char> doc;    // character type defaults to char
    doc.parse<0>(y);    // 0 means default parse flags
    rapidxml::xml_node<char>* objectXMLFile = doc.first_node("SimoxCGAL-SkeletonTool");

    rapidxml::xml_node<>* objectNode = objectXMLFile->first_node("Object", 0, false);

    MeshSkeletonDataPtr data(new MeshSkeletonData());


    //load Object
    if (objectNode)
    {
        rapidxml::xml_node<>* m = objectNode->first_node("File", 0, false);
        rapidxml::xml_attribute<>* m_att = m->first_attribute("file", 0, false);
        string file(m_att->value());
        data->manipObject = ObjectIO::loadManipulationObject(file);

    } else
    {
        THROW_VR_EXCEPTION("No ObjectFile found.\n");
    }


    rapidxml::xml_node<>* nodeMesh = objectXMLFile->first_node("CGAL-Mesh", 0, false);

    //load mesh
    if (nodeMesh)
    {
        rapidxml::xml_node<>* m = nodeMesh->first_node("File", 0, false);
        rapidxml::xml_attribute<>* attribute_file = m->first_attribute("file", 0, false);
        string file(attribute_file->value());
        data->surfaceMesh = CGALMeshIO::LoadSurfaceMesh(file);

    } else
    {
        THROW_VR_EXCEPTION("No mesh found.\n");
    }


    rapidxml::xml_node<>* nodeSkeleton = objectXMLFile->first_node("CGAL-Skeleton", 0, false);

    //load skeleton
    if (nodeSkeleton)
    {
        rapidxml::xml_node<>* m = nodeSkeleton->first_node("File", 0, false);
        rapidxml::xml_attribute<>* attribute_file = m->first_attribute("file", 0, false);
        string file(attribute_file->value());
        SkeletonPtr skeleton = SkeletonIO::loadSkeleton(file);
        data->skeleton = CGALSkeletonPtr(new CGALSkeleton(data->manipObject->getName(), data->surfaceMesh->getMesh(), skeleton));

    }else
    {
        THROW_VR_EXCEPTION("No skeleton found.\n");
    }


    rapidxml::xml_node<>* nodeSegObject = objectXMLFile->first_node("CGAL-SkeletonSegmentation", 0, false);

    //load segmented skeleton
    if (nodeSegObject)
    {
        rapidxml::xml_node<>* m = nodeSegObject->first_node("File", 0, false);
        rapidxml::xml_attribute<>* attribute_file = m->first_attribute("file", 0, false);
        string file(attribute_file->value());
        SegmentedObjectPtr p = SegmentedObjectIO::Load(file);
        string key = "method";
        string value = "skeleton";
        p->addParameterString(key, value);
        data->segSkeleton = MeshSkeletonPtr(new MeshSkeleton(data->surfaceMesh, data->skeleton->getSkeleton(), p));

    }else
    {
        THROW_VR_EXCEPTION("No segmented skeleton found.\n");
    }


    VR_INFO << "Loading complete." << endl;
    delete[] y;
    return data;
}

bool MeshSkeletonData::saveSkeletonData(const string& basePath, const string& objectFile, CGALSkeletonPtr skeleton, CGALSurfaceMeshPtr mesh, SegmentedObjectPtr segmentedObject)
{

    boost::filesystem::path filenameBaseComplete(basePath);
    cout << "Filename: " << filenameBaseComplete.filename() << endl;
    boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
    std::string base = filenameBasePath.string();

    base += "/CGALSkeleton/";

    if (!boost::filesystem::create_directory(base))
    {
        VR_INFO << "Could not create directory. Already exists?" << endl;
    }

    string meshPath = base + "/mesh";
    string skeletonPath = base + "/skeleton";
    string segmentationPath = base + "/segmentation";

    if (!boost::filesystem::create_directory(meshPath))
    {
        VR_INFO << "Could not create directory. Already exists?" << endl;
    }

    if (!boost::filesystem::create_directory(skeletonPath))
    {
        VR_INFO << "Could not create directory. Already exists?" << endl;
    }

    if (!boost::filesystem::create_directory(segmentationPath))
    {
        VR_INFO << "Could not create directory. Already exists?" << endl;
    }

    //createDirectories done

    VR_INFO << "Start saving ... " << endl;

    std::string t = "\t";
    std::stringstream ss;

    string fileMesh = meshPath + "/CGALSkeleton-" + filenameBaseComplete.filename().c_str();
    string fileSkeleton = skeletonPath + "/CGALMesh-" + filenameBaseComplete.filename().c_str();
    string fileSeg = segmentationPath + "/CGALSkeletonSegmentation-" + filenameBaseComplete.filename().c_str();

    bool ok1 = SimoxCGAL::CGALMeshIO::Save(mesh, fileMesh);
    bool ok2 = SkeletonIO::saveSkeletonObject(skeleton, fileSkeleton);
    bool ok3 = SegmentedObjectIO::Save(segmentedObject, fileSeg);

    if (!ok1 || !ok2 || !ok3)
    {
        VR_INFO << "Loading failed. Couldn't save mesh/segmentedObject/skeleton in file." << endl;
        return false;
    }

    ss << "<SimoxCGAL-SkeletonTool>\n";

    ss << t << "<Object>\n";
    ss << t << t << "<File file='" << objectFile << "'/>\n";
    ss << t << "</Object>\n";

    ss << t << "<CGAL-Mesh>\n";
    ss << t << t << "<File file='" << fileMesh << "'/>\n";
    ss << t << "</CGAL-Mesh>\n";

    ss << t << "<CGAL-Skeleton>\n";
    ss << t << t << "<File file='" << fileSkeleton << "'/>\n";
    ss << t << "</CGAL-Skeleton>\n";

    ss << t << "<CGAL-SkeletonSegmentation>\n";
    ss << t << t << "<File file='" << fileSeg << "'/>\n";
    ss << t << "</CGAL-SkeletonSegmentation>\n";

    ss << "</SimoxCGAL-SkeletonTool>\n";

    // save file
    bool ok_save = BaseIO::writeXMLFile(basePath, ss.str(), true);

    if (!ok_save)
    {
        VR_INFO << " ERROR while saving" << endl;
        return false;
    }

    VR_INFO << "Saving complete." << endl;
    return true;
}

}

