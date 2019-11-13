/**
   \file
   \author Ikumi Susa
*/

#include <cnoid/Device>
#include <cnoid/ValueTree>
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/Material>
#include <cnoid/AGXScene>
#include <cnoid/AGXBody>
#include <cnoid/AGXBodyExtension>
#include "../AGXConvert.h"

using namespace std;
namespace cnoid{

const int MAX_NUM_WIRE_NODES_FOR_LOG = 60;

/////////////////////////////////////////////////////////////////////////
// WireNodeState
struct WireNodeState{
    Vector3 position;
};
typedef std::vector<WireNodeState> WireNodeStates;

/////////////////////////////////////////////////////////////////////////
// AGXWireDevice
struct AGXWireDeviceDesc
{
    AGXWireDeviceDesc(){
        radius = 0.2;
    }
    double radius;
};

class AGXWireDevice : private AGXWireDeviceDesc, public Device
{
public:
    static bool createAGXWireDevice(YAMLBodyLoader& loader, Mapping& node);
    AGXWireDevice(const AGXWireDeviceDesc& desc, Mapping* info);
    AGXWireDevice(const AGXWireDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXWireDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void setDesc(const AGXWireDeviceDesc& desc);
    void getDesc(AGXWireDeviceDesc& desc);
    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);
    void   setWireRadius(const double& r);
    double getWireRadius();
    void addWireNodeState(const Vector3& pos);
    WireNodeStates& getWireNodeStates();

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
    
private:
    MappingPtr m_info;
    WireNodeStates m_wireNodeStates;
    AGXWireDevice();
};
typedef ref_ptr<AGXWireDevice> AGXWireDevicePtr;

class AGXBody;
class AGXWire : public AGXBodyExtension
{
public:
    static bool createAGXWire(AGXBody* agxBody);
    AGXWire(AGXWireDevice* device, AGXBody* agxBody);
    void updateWireNodeStates();
private:
    AGXWireDevicePtr m_device;
    agxWire::WireRef m_wire;
    agxWire::WireWinchControllerRef m_winch;
};
typedef ref_ptr<AGXWire> AGXWirePtr;

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
    return new AGXWireDevice(*this, true);
}

Referenced* AGXWireDevice::doClone(CloneMap*) const
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
    return 3 * MAX_NUM_WIRE_NODES_FOR_LOG + 2;
}

const double* AGXWireDevice::readState(const double* buf)
{
    int i = 0;
    const int n = buf[i++];
    m_wireNodeStates.resize(n);
    int j=0;
    while(j < n){
        m_wireNodeStates[j].position << buf[i], buf[i+1], buf[i+2];
        i += 3;
        ++j;
    }
    i += (MAX_NUM_WIRE_NODES_FOR_LOG - j) * 3;
    radius = buf[i++];
    return buf + i;
}

double* AGXWireDevice::writeState(double* out_buf) const
{
    int i = 0;
    const int m = m_wireNodeStates.size();
    int n = std::min(m, MAX_NUM_WIRE_NODES_FOR_LOG);

    out_buf[i++] = n;
    
    int j = 0;
    int offset = (m > n) ? (m - n) : 0;
    while(j < n){
        const Vector3& p = m_wireNodeStates[j + offset].position;
        out_buf[i++] = p.x();
        out_buf[i++] = p.y();
        out_buf[i++] = p.z();
        ++j;
    }
    while(j < MAX_NUM_WIRE_NODES_FOR_LOG){
        out_buf[i++] = 0.0;
        out_buf[i++] = 0.0;
        out_buf[i++] = 0.0;
        ++j;
    }
    
    out_buf[i++] = radius;
    
    return out_buf + i;
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
    const Position::TranslationPart& link_p = m_wireDevice->link()->p();
    const Matrix3& link_attitude_inv = m_wireDevice->link()->attitude().inverse();
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
        p.translation() = pos - link_p;
        p.linear().col(0) = nx;
        p.linear().col(1) = ny;
        p.linear().col(2) = nz;
        sgWireNode->setTransform(link_attitude_inv * p);
    }
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
#define WINCH_READ(FIELD1)   wireWinchInfo.read(#FIELD1, winchDesc.FIELD1)
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
    wireDeviceInfo.read("collision", wireDesc.enableCollisions);
    m_wire = AGXObjectFactory::createWire(wireDesc);

    {   // set Material
		string matName = "";
		agx::Material* mat = nullptr;
		if(wireDeviceInfo.read("materialName", matName)){
            mat = sim->getMaterialManager()->getMaterial(matName);	
		}
        if(mat == nullptr){
            mat = sim->getMaterialManager()->getMaterial(Material::name(0));
        }
        double tmpMatValue;
        if(wireDeviceInfo.read("wireYoungsModulusStretch", tmpMatValue)){
            mat->getWireMaterial()->setYoungsModulusStretch(tmpMatValue);
        }
        if(wireDeviceInfo.read("wireSpookDampingStretch", tmpMatValue)){
            mat->getWireMaterial()->setDampingStretch(tmpMatValue);
        }
        if(wireDeviceInfo.read("wireYoungsModulusBend", tmpMatValue)){
            mat->getWireMaterial()->setYoungsModulusBend(tmpMatValue);
        }
        if(wireDeviceInfo.read("wireSpookDampingBend", tmpMatValue)){
            mat->getWireMaterial()->setDampingBend(tmpMatValue);
        }
        m_wire->setMaterial(mat);
    }

    // create winch
    const ValueNodePtr& wireWinchVNPtr = wireDeviceInfo.find("Winch");
    if(wireWinchVNPtr->isMapping()){
        const Mapping&  wireWinchInfo = *wireWinchVNPtr->toMapping();
        AGXWireWinchControllerDesc winchDesc;
        string linkName;
        wireWinchInfo.read("linkName", linkName);
        AGXLink* const agxLink = agxBody->getAGXLink(linkName);
        const Matrix3& attitude = agxLink->getOrgLink()->attitude();

        winchDesc.rigidBody = agxBody->getAGXRigidBody(linkName);

        Vector3 positionInBodyFrame;
        agxConvert::setVector(wireWinchInfo.find("position"), positionInBodyFrame);
        winchDesc.positionInBodyFrame = agxConvert::toAGX(positionInBodyFrame);   // link coord

        Vector3 normalInBodyFrame;
        agxConvert::setVector(wireWinchInfo.find("normal"), normalInBodyFrame);
        winchDesc.normalInBodyFrame = agxConvert::toAGX(attitude * normalInBodyFrame);      // link coord

        wireWinchInfo.read("pulledInLength", winchDesc.pulledInLength);
        m_winch = AGXObjectFactory::createWinchController(winchDesc);
        if(m_winch){
            std::vector<string> haulForceRange;
            if(agxConvert::setVector(wireWinchInfo.find("haulForceRange"), 2, haulForceRange)){
                m_winch->setForceRange(agx::RangeReal(std::stod(haulForceRange[0]), std::stod(haulForceRange[1])));
            }
            m_wire->add(m_winch);
        }
    }

    // set wire node
    const ValueNodePtr& wireNodesInfo = wireDeviceInfo.find("Nodes");
    if(wireNodesInfo->isListing()){
        const Listing& wireNodeList = *wireNodesInfo->toListing();
        for(const auto& wireNode : wireNodeList){
            if(!wireNode->isMapping()) continue;
            const Mapping&  wireNodeInfo = *wireNode->toMapping();
            string nodeType, coordinate, linkName;
            Vector3 pos;
            // Read yaml
            wireNodeInfo.read("type", nodeType);
            wireNodeInfo.read("linkName", linkName);
            agxConvert::setVector(wireNodeInfo.find("position"), pos);

            auto transformToWorld = [&](){
                // Transform pos from link coord to world coord, if link exist
                if(Link* const link = getAGXBody()->body()->link(linkName)){
                    pos = link->p() + link->attitude() * pos;
                }
                return agxConvert::toAGX(pos);
            };

            if(nodeType == "free"){
                // set in world coord
                m_wire->add(AGXObjectFactory::createWireFreeNode(transformToWorld()));
            }else if(nodeType == "fixed"){
                if(Link* const link = getAGXBody()->body()->link(linkName)){
                    // set in link coord
                    agx::RigidBody* body = getAGXBody()->getAGXRigidBody(linkName);
                    // reflect pose correction
                    pos = link->Rs() * pos;
                    m_wire->add(AGXObjectFactory::createWireBodyFixedNode(body, agxConvert::toAGX(pos)));
                }else{
                    // set in world coord
                    m_wire->add(AGXObjectFactory::createWireBodyFixedNode(nullptr, transformToWorld()));
                }
            }else if(nodeType == "link"){
                agx::RigidBody* wireLinkBody = getAGXBody()->getAGXRigidBody(linkName);
                agxWire::LinkRef wireLink = AGXObjectFactory::createWireLink(wireLinkBody);
                // set in link coord
                m_wire->add(wireLink, agxConvert::toAGX(pos));
                agxWire::ILinkNodeRef iLinkNode = wireLink->getConnectingNode(m_wire);
                if(iLinkNode){
                    auto getILinkNodeValue = [&](string key, double defaultValue){
                        return wireNodeInfo.get(key, defaultValue);
                    };
                    agxWire::ILinkNode::ConnectionProperties* cp = iLinkNode->getConnectionProperties();
                    cp->setTwistStiffness(getILinkNodeValue("twistStiffness", cp->getTwistStiffness()));
                    cp->setBendStiffness(getILinkNodeValue("bendStiffness", cp->getBendStiffness()));
                    iLinkNode->setSuperBendReplacedWithBend(getILinkNodeValue("superBendReplacedWithBend", iLinkNode->getSuperBendReplacedWithBend()));
                    m_wire->setEnableCollisions(wireLinkBody, false);
                }
            }
        }
    }

    // add wire to simulation
    sim->add((agxSDK::StepEventListener*)m_wire);
    agxWire::WireController::instance()->setEnableCollisions(m_wire, m_wire, wireDeviceInfo.get("selfCollision", false));

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
