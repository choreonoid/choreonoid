#include "StdBodyWriter.h"
#include <cnoid/StdSceneWriter>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/SceneGraph>
#include <cnoid/YAMLWriter>
#include <cnoid/EigenArchive>
#include <cnoid/stdx/filesystem>

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class StdBodyWriter::Impl
{
public:
    StdSceneWriter sceneWriter;
    YAMLWriter yamlWriter;

    Impl();
    MappingPtr writeBody(Body* body);
    MappingPtr writeLink(Link* link);
};

}


StdBodyWriter::StdBodyWriter()
{
    impl = new Impl;
}


StdBodyWriter::Impl::Impl()
{
    yamlWriter.setKeyOrderPreservationMode(true);
}


bool StdBodyWriter::writeBody(Body* body, const std::string& filename)
{
    if(impl->yamlWriter.openFile(filename)){
        auto directory = filesystem::path(filename).parent_path().generic_string();
        impl->sceneWriter.setBaseDirectory(directory);
        impl->yamlWriter.putNode(impl->writeBody(body));
        impl->yamlWriter.closeFile();
        return true;
    }
    return false;
}


MappingPtr StdBodyWriter::Impl::writeBody(Body* body)
{
    MappingPtr node = new Mapping;

    node->write("format", "ChoreonoidBody");
    node->write("formatVersion", "1.0");

    node->write("name", body->name(), DOUBLE_QUOTED);
    node->write("rootLink", body->rootLink()->name(), DOUBLE_QUOTED);

    ListingPtr linksNode = new Listing;
    for(auto& link : body->links()){
        linksNode->append(writeLink(link));
    }
    if(!linksNode->empty()){
        node->insert("links", linksNode);
    }
    
    return node;
}


MappingPtr StdBodyWriter::Impl::writeLink(Link* link)
{
    MappingPtr node = new Mapping;

    node->write("name", link->name(), DOUBLE_QUOTED);
    node->write("jointType", link->jointTypeString());
    node->write("mass", link->mass());
    write(*node, "centerOfMass", link->centerOfMass());
    write(*node, "inertia", link->I());

    ListingPtr elementsNode = new Listing;

    if(!link->hasDedicatedCollisionShape()){
        for(auto& shape : *link->shape()){
            if(auto shapeNode = sceneWriter.writeScene(shape)){
                elementsNode->append(shapeNode);
            }
        }
    }

    if(!elementsNode->empty()){
        node->insert("elements", elementsNode);
    }

    return node;
}
