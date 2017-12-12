/**
   \file
   \author Ikumi Susa
*/

#include "AGXWire.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/AGXBody>
#include <cnoid/AGXScene>
#include "../AGXConvert.h"
#include <cnoid/Material>

using namespace std;
namespace cnoid{

/////////////////////////////////////////////////////////////////////////
// SceneWireDevice
class SceneWireDevice : public SceneDevice
{
public:
    static SceneDevice* createSceneWireDevice(Device* device);
    SceneWireDevice(AGXWireDevice* device);
    void update();
private:
    AGXWireDevice* m_wireDevice;
};

SceneDevice* SceneWireDevice::createSceneWireDevice(Device* device)
{
    return new SceneWireDevice(static_cast<AGXWireDevice*>(device));
}

SceneWireDevice::SceneWireDevice(AGXWireDevice* device) :
    SceneDevice(device),
    m_wireDevice(device)
{
    setFunctionOnStateChanged([&](){ update(); });
}

void SceneWireDevice::update()
{
    clearChildren();
    if(m_wireDevice->getWireNodeStates().size() <= 0) return;
    MeshGenerator meshGenerator;
    const double& radius = m_wireDevice->getWireRadius();
    const Position& linkPos_inv = m_wireDevice->link()->T().inverse();
    const int& numNodes = (int)m_wireDevice->getWireNodeStates().size();
    for(size_t i = 0; i < numNodes - 1; ++i){
        const WireNodeState& node1 = m_wireDevice->getWireNodeStates()[i];
        const WireNodeState& node2 = m_wireDevice->getWireNodeStates()[i + 1];
        SgPosTransformPtr sgWireNode = new SgPosTransform;
        addChild(sgWireNode, true);

        // Add shape
        Vector3 dir = node2.position.operator-(node1.position);
        double length = dir.norm();
        auto shape = new SgShape;
        shape->setMesh(meshGenerator.generateCapsule(radius, length));
        sgWireNode->addChild(shape, true);

        // Set transform
        Vector3 pos = node2.position.operator+(node1.position) * 0.5;
        dir.normalize();
        const Vector3& ny = dir;
        Vector3 nx = Vector3(1.0, 0.0, 0.0).cross(ny);
        if(nx.norm() < 1.0e-6){
            nx = Vector3(0.0, 1.0, 0.0).cross(ny);
        }
        nx.normalize();
        const Vector3 nz = nx.cross(ny);

        Position p;
        p.setIdentity();
        p.translation() = pos;
        p.linear().col(0) = nx;
        p.linear().col(1) = ny;
        p.linear().col(2) = nz;
        sgWireNode->setTransform(linkPos_inv * p);

        //Eigen::IOFormat fmt(4, 0, ", ", "\n", "", "");
        //std::cout << "wire translation " << sgWireNode->translation() << std::endl;
        //Affine3 af;
        //sgWireNode->getTransform(af);
        //std::cout << "wire transform " << af.matrix().format(fmt) << std::endl;
    }
}

/////////////////////////////////////////////////////////////////////////
// AGXWireDevice
bool AGXWireDevice::createAGXWireDevice(YAMLBodyLoader&loader, Mapping& node)
{
    MappingPtr info = node.cloneMapping();

    // Wire
    AGXWireDeviceDesc desc;
    AGXWireDevicePtr wireDevice = new AGXWireDevice(desc, info);

    return loader.readDevice(wireDevice, node);
}

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
    auto& device = const_cast<AGXWireDevice&>(other);
    device.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    resetInfo(device.info());
    m_wireNodeStates = other.m_wireNodeStates;
}

void AGXWireDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AGXWireDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(dynamic_cast<const AGXWireDevice&>(other));
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

void AGXWireDevice::setWireRadius(const double& r)
{
    radius = r;
}

double AGXWireDevice::getWireRadius()
{
    return radius;
}

void AGXWireDevice::addWireNodeState(const Vector3& pos)
{
    WireNodeState s;
    s.position = pos;
    m_wireNodeStates.push_back(s);
}

WireNodeStates& AGXWireDevice::getWireNodeStates()
{
    return m_wireNodeStates;
}

/////////////////////////////////////////////////////////////////////////
// WireListener
class WireListener : public agxSDK::StepEventListener
{
public:
    WireListener(AGXWire* agxWire) :
        m_agxWire(agxWire)
    {
        setMask( POST_STEP );
    }

    virtual void post( const agx::TimeStamp& /*t*/ )
    {
        m_agxWire->updateWireNodeStates();
    }

private:
    AGXWire* m_agxWire;
};

/////////////////////////////////////////////////////////////////////////
// AGXWire
bool AGXWire::createAGXWire(cnoid::AGXBody* agxBody)
{
    DeviceList<> devices = agxBody->body()->devices();
    DeviceList<AGXWireDevice> wireDevices;
    wireDevices.extractFrom(devices);
    for(auto& device : wireDevices){
        agxBody->addAGXBodyExtension(new cnoid::AGXWire(device, agxBody));
    }
    return true;
};

#define NODE_READ(FIELD1)    wireDeviceInfo.read(#FIELD1, wireDesc.FIELD1)
AGXWire::AGXWire(AGXWireDevice* device, AGXBody* agxBody) :
    AGXBodyExtension(agxBody),
    m_device(device)
{
    agxSDK::SimulationRef sim = agxBody->getAGXScene()->getSimulation();
    Mapping& wireDeviceInfo = *m_device->info();

    // create wire
    AGXWireDesc wireDesc;
    NODE_READ(radius);
    m_device->setWireRadius(wireDesc.radius);
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
    double tmpValue;
    if(wireDeviceInfo.read("wireYoungsModulusStretch", tmpValue)){
        mat->getWireMaterial()->setYoungsModulusStretch(tmpValue);
    }
    if(wireDeviceInfo.read("wireDampingStretch", tmpValue)){
        mat->getWireMaterial()->setDampingStretch(tmpValue);
    }
    if(wireDeviceInfo.read("wireYoungsModulusBend", tmpValue)){
        mat->getWireMaterial()->setYoungsModulusBend(tmpValue);
    }
    if(wireDeviceInfo.read("wireDampingBend", tmpValue)){
        mat->getWireMaterial()->setDampingBend(tmpValue);
    }
    m_wire->setMaterial(mat);
    sim->add((agxSDK::StepEventListener*)m_wire);


    // create winch
    AGXLink* link = agxBody->getAGXLink(m_device->link()->name());
    if(link){
        AGXWireWinchControllerDesc winchDesc;
        winchDesc.rigidBody = link->getAGXRigidBody();
        //winchDesc.positionInBodyFrame = ddesc.winchPosition;
        std::string winchLinkName;
        Vector3 winchPosition;
        Vector3 winchDirection;
        double  winchPulledInLenght;
    }

    // Rendering
    sim->add(new WireListener(this));
    updateWireNodeStates();
    m_device->notifyStateChange();
}
#undef NODE_READ

void AGXWire::updateWireNodeStates()
{
    m_device->getWireNodeStates().clear();
    agxWire::RenderIterator it = m_wire->getRenderBeginIterator();
    const agxWire::RenderIterator end = m_wire->getRenderEndIterator();
    while ( it != end ) {
        const agxWire::Node* node = *it;
        const Vector3 pos = agxConvert::toCnoid(node->getWorldPosition());
        m_device->addWireNodeState(pos);
        ++it;
    }
    m_device->notifyStateChange();
}

/////////////////////////////////////////////////////////////////////////
// Register Device
struct AGXWireDeviceRegistration
{
    AGXWireDeviceRegistration(){
        cnoid::YAMLBodyLoader::addNodeType("AGXWireDevice", AGXWireDevice::createAGXWireDevice);
        SceneDevice::registerSceneDeviceFactory<AGXWireDevice>(SceneWireDevice::createSceneWireDevice);
        if(AGXObjectFactory::checkModuleEnalbled("AgX-Wires")){
        }else {
            std::cout << "Please check you have AgX-Wires module license."
            "AGXWireDevice should not work." << std::endl;
        }
    }
}registrationAGXWireDevice;

/////////////////////////////////////////////////////////////////////////
// Register AGXWire
struct AGXWireRegistration
{
    AGXWireRegistration() {
        cnoid::AGXBody::addAGXBodyExtensionAdditionalFunc("AGXWire", AGXWire::createAGXWire);
    }
}registrationAGXWireRegistration;

}