/**
   \file
   \author Ikumi Susa
*/

#include "AGXWire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/AGXBody>
#include <cnoid/AGXScene>
#include "../AGXConvert.h"
#include <cnoid/Material>

using namespace std;
namespace cnoid{

/////////////////////////////////////////////////////////////////////////
// Register Device
#define NODE_READ(FIELD1)    node.read(#FIELD1, desc.FIELD1)
#define NODE_READ_VEC3(FIELD1)   agxConvert::setVector(node.find(#FIELD1), desc.FIELD1);
bool readAGXWireDevice(YAMLBodyLoader&loader, Mapping& node)
{
    MappingPtr info = node.cloneMapping();

    // Wire
    AGXWireDeviceDesc desc;
    AGXWireDevicePtr wireDevice = new AGXWireDevice(desc, info);

    return loader.readDevice(wireDevice, node);
}
#undef NODE_READ
#undef NODE_READ_VEC3

struct AGXWireDeviceRegistration
{
    AGXWireDeviceRegistration(){
        cnoid::YAMLBodyLoader::addNodeType("AGXWireDevice", readAGXWireDevice);
    }
}registrationAGXWireDevice;


/////////////////////////////////////////////////////////////////////////
// AGXWireDevice

AGXWireDevice::AGXWireDevice(const AGXWireDeviceDesc& desc, Mapping* info) :
AGXWireDeviceDesc(desc)
{
    resetInfo(info);
}

AGXWireDevice::AGXWireDevice(const AGXWireDevice& org, bool copyStateOnly) :
Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char*AGXWireDevice::typeName()
{
    return "AGXWireDevice";
}

void AGXWireDevice::copyStateFrom(const AGXWireDevice& other)
{
    AGXWireDeviceDesc desc;
    AGXWireDevice& device = const_cast<AGXWireDevice&>(other);
    device.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    resetInfo(device.info());
}

void AGXWireDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AGXWireDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXWireDevice&>(other));
}

DeviceState* AGXWireDevice::cloneState() const
{
    return new AGXWireDevice(*this, false);
}

Device*AGXWireDevice::clone() const
{
    return new AGXWireDevice(*this);
}

void AGXWireDevice::forEachActualType(std::function<bool(const std::type_info&type)> func)
{
    if(!func(typeid(AGXWireDevice))){
        Device::forEachActualType(func);
    }
}

int AGXWireDevice::stateSize() const
{
    return 1;
}

const double* AGXWireDevice::readState(const double* buf)
{
    return buf + 1;
}

double* AGXWireDevice::writeState(double* out_buf) const
{
    return out_buf + 1;
}

void AGXWireDevice::setDesc(const AGXWireDeviceDesc& desc)
{
    static_cast<AGXWireDeviceDesc&>(*this) = desc;
}

void AGXWireDevice::getDesc(AGXWireDeviceDesc& desc)
{
    desc = static_cast<AGXWireDeviceDesc&>(*this);
}

const Mapping* AGXWireDevice::info() const
{
    return m_info;
}

Mapping* AGXWireDevice::info()
{
    return m_info;
}

void AGXWireDevice::resetInfo(Mapping* info)
{
    m_info = info;
}

/////////////////////////////////////////////////////////////////////////
// Register AGXWire
bool createAGXWire(cnoid::AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXWireDevice> wireDevices;
    wireDevices.extractFrom(devices);
    for(auto device : wireDevices){
        agxBody->addAGXBodyExtension(new cnoid::AGXWire(device, agxBody));
    }
    return true;
};

struct AGXWireRegistration
{
    AGXWireRegistration() {
        cnoid::AGXBody::addAGXBodyExtensionAdditionalFunc("AGXWire", createAGXWire);
    }
};
AGXWireRegistration registrationAGXWireRegistration;

/////////////////////////////////////////////////////////////////////////
// AGXWire
#define NODE_READ(FIELD1)    wireDeviceInfo.read(#FIELD1, wireDesc.FIELD1)
AGXWire::AGXWire(AGXWireDevice* device, AGXBody* agxBody) :
AGXBodyExtension(agxBody)
{
    Mapping& wireDeviceInfo = *device->info();

    agxSDK::SimulationRef sim = agxBody->getAGXScene()->getSimulation();
    AGXWireDeviceDesc ddesc;
    device->getDesc(ddesc);

    // create wire
    AGXWireDesc wireDesc;
    NODE_READ(radius);
    NODE_READ(resolutionPerUnitLength);
    NODE_READ(enableCollisions);
    m_wire = AGXObjectFactory::createWire(wireDesc);

    // set wire node
    const ValueNodePtr& wireNodesInfo = wireDeviceInfo.find("Nodes");
    if(wireNodesInfo->isListing()){
        const Listing& wireNodeList = *wireNodesInfo->toListing();
        for(const auto& wireNode : wireNodeList){
            if(!wireNode->isMapping()) continue;
            const Mapping&  wireNodeInfo = *wireNode->toMapping();
            string nodeType;
            Vector3 pos;
            wireNodeInfo.read("type", nodeType);
            agxConvert::setVector(wireNodeInfo.find("position"), pos);
            if(nodeType == "free"){
                m_wire->add(AGXObjectFactory::createWireFreeNode(agxConvert::toAGX(pos)));
            }else if(nodeType == "fixed"){

            }
        }
    }

    agx::Material* mat = sim->getMaterialManager()->getMaterial(wireDeviceInfo.read<string>("materialName"));
    if(mat == nullptr){
        mat = sim->getMaterialManager()->getMaterial(Material::name(0));
    }
    m_wire->setMaterial(mat);
    sim->add(m_wire);


    // create winch
    AGXLink* link = agxBody->getAGXLink(device->link()->name());
    if(link){
        AGXWireWinchControllerDesc winchDesc;
        winchDesc.rigidBody = link->getAGXRigidBody();
        //winchDesc.positionInBodyFrame = ddesc.winchPosition;
        std::string winchLinkName;
        Vector3 winchPosition;
        Vector3 winchDirection;
        double  winchPulledInLenght;
    }

}

} // namespace cnoid