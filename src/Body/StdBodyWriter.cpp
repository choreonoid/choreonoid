#include "StdBodyWriter.h"
#include <cnoid/StdSceneWriter>
#include <cnoid/Body>
#include <cnoid/Link>
#include <cnoid/BodyHandler>
#include <cnoid/SceneGraph>
#include <cnoid/YAMLWriter>
#include <cnoid/EigenArchive>
#include <cnoid/NullOut>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class StdBodyWriter::Impl
{
public:
    StdSceneWriter sceneWriter;
    YAMLWriter yamlWriter;

    ostream* os_;
    ostream& os() { return *os_; }

    Impl();
    bool writeBody(Body* body, const std::string& filename);
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
    sceneWriter.setExtModelFileMode(StdSceneWriter::EmbedModels);
    yamlWriter.setKeyOrderPreservationMode(true);
    os_ = &nullout();
}


void StdBodyWriter::setMessageSink(std::ostream& os)
{
    impl->os_ = &os;
    impl->sceneWriter.setMessageSink(os);
}


void StdBodyWriter::setExtModelFileMode(int mode)
{
    switch(mode){
    case EmbedModels:
        impl->sceneWriter.setExtModelFileMode(StdSceneWriter::EmbedModels);
        break;
    case LinkToOriginalModelFiles:
        impl->sceneWriter.setExtModelFileMode(StdSceneWriter::LinkToOriginalModelFiles);
        break;
    case ReplaceWithStdSceneFiles:
        impl->sceneWriter.setExtModelFileMode(StdSceneWriter::ReplaceWithStdSceneFiles);
        break;
    case ReplaceWithObjModelFiles:
        impl->sceneWriter.setExtModelFileMode(StdSceneWriter::ReplaceWithObjModelFiles);
        break;
    default:
        break;
    }
}


int StdBodyWriter::extModelFileMode() const
{
    switch(impl->sceneWriter.extModelFileMode()){
    case StdSceneWriter::EmbedModels: return EmbedModels;
    case StdSceneWriter::LinkToOriginalModelFiles: return LinkToOriginalModelFiles;
    case StdSceneWriter::ReplaceWithStdSceneFiles: return ReplaceWithStdSceneFiles;
    case StdSceneWriter::ReplaceWithObjModelFiles: return ReplaceWithObjModelFiles;
    default: return -1;
    }
}


bool StdBodyWriter::writeBody(Body* body, const std::string& filename)
{
    return impl->writeBody(body, filename);
}


bool StdBodyWriter::Impl::writeBody(Body* body, const std::string& filename)
{
    bool result = false;
    
    auto directory = filesystem::path(filename).parent_path().generic_string();
    sceneWriter.setBaseDirectory(directory);
    
    auto topNode = writeBody(body);

    if(topNode){
        if(yamlWriter.openFile(filename)){
            yamlWriter.putNode(topNode);
            result = true;
            yamlWriter.closeFile();
        }
    }
    
    return result;
}


MappingPtr StdBodyWriter::Impl::writeBody(Body* body)
{
    MappingPtr node = new Mapping;

    node->write("format", "ChoreonoidBody");
    node->write("format_version", "2.0");
    node->write("angle_unit", "degree");

    node->write("name", body->name(), DOUBLE_QUOTED);
    node->write("root_link", body->rootLink()->name(), DOUBLE_QUOTED);

    ListingPtr linksNode = new Listing;
    for(auto& link : body->links()){
        linksNode->append(writeLink(link));
    }
    if(!linksNode->empty()){
        node->insert("links", linksNode);
    }

    int numBodyHandlers = body->numHandlers();
    if(numBodyHandlers > 0){
        ListingPtr handlers = new Listing;
        for(int i=0; i < numBodyHandlers; ++i){
            auto& filename = body->handler(0)->filename();
            if(!filename.empty()){
                handlers->append(filename);
            }
        }
        if(handlers->size() == 1){
            node->write("body_handlers", handlers->front()->toString());
        } else if(handlers->size() >= 2){
            node->insert("body_handlers", handlers);
        }
    }

    node->insert(body->info());
    
    return node;
}


MappingPtr StdBodyWriter::Impl::writeLink(Link* link)
{
    MappingPtr node = new Mapping;

    if(link->name().empty()){
        os() << format(_("The name of the link {0} is not specified.")) << endl;
        return nullptr;
    }
    node->write("name", link->name(), DOUBLE_QUOTED);

    if(auto parent = link->parent()){
        node->write("parent", link->parent()->name());
    }

    auto b = link->offsetTranslation();
    if(!b.isZero()){
        write(node, "translation", b);
    }
    auto aa = AngleAxis(link->offsetRotation());
    if(aa.angle() != 0.0){
        writeDegreeAngleAxis(node, "rotation", aa);
    }

    node->write("joint_type", link->jointTypeString());

    if(!link->isFreeJoint() && !link->isFixedJoint()){
        write(node, "joint_axis", link->jointAxis());
    }

    if(link->jointId() >= 0){
        node->write("joint_id", link->jointId());
    }
    if(link->hasJoint()){
        auto rangeNode = node->createFlowStyleListing("joint_range");
        if(link->isRevoluteJoint()){
            rangeNode->append(degree(link->q_lower()));
            rangeNode->append(degree(link->q_upper()));
        } else {
            rangeNode->append(link->q_lower());
            rangeNode->append(link->q_upper());
        }
    }
    
    node->write("mass", link->mass());
    write(node, "center_of_mass", link->centerOfMass());
    write(node, "inertia", link->I());

    node->insert(link->info());

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
