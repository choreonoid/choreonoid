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
#include <mutex>
#include <typeindex>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

std::mutex deviceWriterRegistrationMutex;

struct WriterInfo
{
    std::string typeName;
    std::function<bool(StdBodyWriter* writer, Mapping* info, const Device* device)> func;
    WriterInfo(
        const char* typeName,
        const std::function<bool(StdBodyWriter* writer, Mapping* info, const Device* device)>& func)
        : typeName(typeName), func(func) { }
};
        
map<std::type_index, WriterInfo> registeredDeviceWriterMap;

}

namespace cnoid {

class StdBodyWriter::Impl
{
public:
    StdBodyWriter* self;
    StdSceneWriter sceneWriter;
    YAMLWriter yamlWriter;
    map<std::type_index, WriterInfo> deviceWriterMap;
    map<int, vector<Device*>> linkIndexToDeviceListMap;

    ostream* os_;
    ostream& os() { return *os_; }

    Impl(StdBodyWriter* self);
    void updateDeviceWriteFunctions();
    bool writeBody(Body* body, const std::string& filename);
    MappingPtr writeBody(Body* body);
    MappingPtr writeLink(Link* link);
    void writeLinkShape(Listing* elementsNode, SgGroup* shapeGroup, const char* type);
    void writeLinkDevices(Listing* elementsNode, Link* link);
    MappingPtr writeDevice(const std::string& typeName, Device* device);
};

}

StdBodyWriter::StdBodyWriter()
{
    impl = new Impl(this);
}


StdBodyWriter::Impl::Impl(StdBodyWriter* self)
    : self(self)
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

    updateDeviceWriteFunctions();

    for(auto& device : body->devices()){
        linkIndexToDeviceListMap[device->link()->index()].push_back(device);
    }

    auto topNode = writeBody(body);

    if(topNode){
        if(yamlWriter.openFile(filename)){
            yamlWriter.putNode(topNode);
            result = true;
            yamlWriter.closeFile();
        }
    }
    
    linkIndexToDeviceListMap.clear();
    
    return result;
}


void StdBodyWriter::Impl::updateDeviceWriteFunctions()
{
    std::lock_guard<std::mutex> guard(deviceWriterRegistrationMutex);
    if(deviceWriterMap.size() < registeredDeviceWriterMap.size()){
        deviceWriterMap.insert(
            registeredDeviceWriterMap.begin(), registeredDeviceWriterMap.end());
    }
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

    node->write("joint_type", link->jointTypeSymbol());

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
        writeLinkShape(elementsNode, link->shape(), nullptr);
    } else {
        writeLinkShape(elementsNode, link->visualShape(), "Visual");
        writeLinkShape(elementsNode, link->collisionShape(), "Collision");
    }

    writeLinkDevices(elementsNode, link);

    if(!elementsNode->empty()){
        node->insert("elements", elementsNode);
    }

    return node;
}


void StdBodyWriter::Impl::writeLinkShape(Listing* elementsNode, SgGroup* shapeGroup, const char* type)
{
    if(shapeGroup->empty()){
        return;
    }
    MappingPtr typeNode;
    Listing* subElementsNode;
    if(type){
        typeNode = new Mapping;
        typeNode->write("type", type);
        subElementsNode = typeNode->createListing("elements");
    } else {
        subElementsNode = elementsNode;
    }
    for(auto& shape : *shapeGroup){
        if(auto shapeNode = sceneWriter.writeScene(shape)){
            subElementsNode->append(shapeNode);
        }
    }
    if(type){
        if(!subElementsNode->empty()){
            elementsNode->append(typeNode);
        }
    }
}


void StdBodyWriter::Impl::writeLinkDevices(Listing* elementsNode, Link* link)
{
    auto p = linkIndexToDeviceListMap.find(link->index());
    if(p == linkIndexToDeviceListMap.end()){
        return;
    }
    auto& devices = p->second;
    for(auto& device : devices){
        auto q = deviceWriterMap.find(typeid(*device));
        if(q == deviceWriterMap.end()){
            if(device->name().empty()){
                os() << format(_("Warning: {0} of {1} cannot be written "
                                 "because its writing function is not provided."),
                               device->typeName(), link->name()) << endl;
            } else {
                os() << format(_("Warning: Device \"{0}\" of the {1} type cannot be written "
                                 "because its writing function is not provided."),
                               device->name(), device->typeName()) << endl;
            }
        } else {
            auto& writer = q->second;
            MappingPtr node = writeDevice(writer.typeName, device);
            if(writer.func(self, node, device)){
                elementsNode->append(node);
            } else {
                if(device->name().empty()){
                    os() << format(_("Warning: Writing {0} of {1} failed."),
                                   device->typeName(), link->name()) << endl;
                } else {
                    os() << format(_("Warning: Writing device \"{0}\" of the {1} type failed."),
                                   device->name(), device->typeName()) << endl;
                }
            }
        }
    }
}


MappingPtr StdBodyWriter::Impl::writeDevice(const std::string& typeName, Device* device)
{
    MappingPtr info = new Mapping;

    info->write("type", typeName);
    
    if(!device->name().empty()){
        info->write("name", device->name());
    }
    if(device->id() >= 0){
        info->write("id", device->id());
    }
    auto& p = device->localTranslation();
    if(!p.isZero()){
        write(info, "translation", p);
    }
    auto aa = AngleAxis(device->localRotation());
    if(aa.angle() != 0.0){
        writeDegreeAngleAxis(info, "rotation", aa);
    }
    
    return info;
}


StdSceneWriter* StdBodyWriter::sceneWriter()
{
    return &impl->sceneWriter;
}


const StdSceneWriter* StdBodyWriter::sceneWriter() const
{
    return &impl->sceneWriter;
}


void StdBodyWriter::registerDeviceWriter_
(const std::type_info& type, const char* typeName,
 std::function<bool(StdBodyWriter* writer, Mapping* info, const Device* device)> writeFunction)
{
    std::lock_guard<std::mutex> guard(deviceWriterRegistrationMutex);
    registeredDeviceWriterMap.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(type),
        std::forward_as_tuple(typeName, writeFunction));
}


bool StdBodyWriter::writeDeviceAs_(const std::type_info& type, Mapping* info, const Device* device)
{
    auto p = impl->deviceWriterMap.find(type);
    if(p != impl->deviceWriterMap.end()){
        auto& writer = p->second;
        return writer.func(this, info, device);
    }
    return true;
}
