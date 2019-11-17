#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/YAMLSceneReader>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include "AGXObjectFactory.h"
#include <cnoid/Link>

using namespace std;

namespace cnoid {

class SceneTrackDevice : public SceneDevice
{
    AGXVehicleContinuousTrackDevice* m_trackDevice;
    std::vector<SgPosTransformPtr> m_sgTracks;
    SgSwitchPtr trackSwitch;

public:
    SceneTrackDevice(AGXVehicleContinuousTrackDevice* device)
        : SceneDevice(device)
    {
        m_trackDevice = device;
        trackSwitch = new SgSwitch;
        trackSwitch->turnOff();
        addChild(trackSwitch);
        setFunctionOnStateChanged([&](){ update(); });
        initialize();
    }

    void initialize(){
        m_sgTracks.clear();
        m_sgTracks.reserve(m_trackDevice->getNumNodes());
        for(int i = 0; i < m_trackDevice->getNumNodes(); ++i){
            SgPosTransformPtr sgTrack = new SgPosTransform;
            trackSwitch->addChild(sgTrack);
            m_sgTracks.push_back(sgTrack);
        }
    }

    void update(){
        if(m_trackDevice->getTrackStates().size() <= 0){
            trackSwitch->setTurnedOn(false, true);
        } else {
            if(!trackSwitch->isTurnedOn()){
                createTrackShapes();
                trackSwitch->setTurnedOn(true, true);
            }
            updateTrackPosition();
        }
    }

    void createTrackShapes(){
        MeshGenerator meshGenerator;
        const TrackStates& states = m_trackDevice->getTrackStates();
        for(size_t i = 0; i < states.size(); ++i) {
            m_sgTracks[i]->clearChildren();
            const auto& size = states[i].boxSize;
            if(size != Vector3::Zero()){
                auto shape = m_trackDevice->getNodeShape();
                if(shape){
                    shape = dynamic_cast<SgShape*>(shape->clone());
                } else {
                    shape = new SgShape;
                }
                shape->setMesh(meshGenerator.generateBox(size));
                m_sgTracks[i]->addChild(shape, true);
            }
        }
    }

    void updateTrackPosition() {
        const Position &linkPos_inv = m_trackDevice->link()->T().inverse();
        const TrackStates& states = m_trackDevice->getTrackStates();
        for(size_t i = 0; i < states.size(); ++i){
            const auto& state = states[i];
            if(state.boxSize.x() == 0.0){
                trackSwitch->turnOff(true);
                break;
            } else {
                m_sgTracks[i]->setTransform(linkPos_inv * state.position);
            }
        }
    }
};

#define NODE_READ(FIELD1) node.read(#FIELD1, desc.FIELD1)
bool readAGXVehicleContinuousTrackDevice(YAMLBodyLoader& loader, Mapping& node)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    if(!NODE_READ(numberOfNodes)) return false;
    if(!NODE_READ(nodeThickness)) return false;
    if(!NODE_READ(nodeWidth)) return false;
    NODE_READ(nodeDistanceTension);
    NODE_READ(nodeThickerThickness);
    NODE_READ(useThickerNodeEvery);
    NODE_READ(hingeCompliance);
    node.read("hingeSpookDamping", desc.hingeSpookDamping);
    NODE_READ(minStabilizingHingeNormalForce);
    NODE_READ(stabilizingHingeFrictionParameter);
    NODE_READ(nodesToWheelsMergeThreshold);
    NODE_READ(nodesToWheelsSplitThreshold);
    NODE_READ(enableMerge);
    NODE_READ(numNodesPerMergeSegment);
    NODE_READ(contactReduction);
    NODE_READ(enableLockToReachMergeCondition);
    NODE_READ(lockToReachMergeConditionCompliance);
    node.read("lockToReachMergeConditionSpookDamping", desc.lockToReachMergeConditionSpookDamping);
    NODE_READ(maxAngleMergeCondition);
    node.read("material", desc.materialName);

    desc.nodeShape = dynamic_cast<SgShape*>(       
        loader.sceneReader().readNode(*node.findMapping("nodeShape"), "Shape"));

    // Get name of wheels from yaml
    const auto toVectorString = [](ValueNodePtr const vnptr, vector<string>& vs) ->bool
    {
        if(!vnptr) return false;
        Listing& list = *vnptr->toListing();
        for(int i=0; i < list.size(); i++){
            vs.push_back(list[i].toString());
        }
        return true;
    };
    MappingPtr info = static_cast<Mapping*>(node.clone());
    toVectorString(info->extract("sprocketNames"), desc.sprocketNames);
    toVectorString(info->extract("idlerNames"), desc.idlerNames);
    toVectorString(info->extract("rollerNames"), desc.rollerNames);
    toVectorString(info->extract("guideNames"), desc.guideNames);
    ValueNodePtr const upAxis = info->extract("upAxis");
    if(upAxis){
        Listing& u = *upAxis->toListing();
        if(u.size() != 3) return false;
        desc.upAxis = Vector3(u[0].toDouble(), u[1].toDouble(), u[2].toDouble());
    }else{
        return false;
    }
    AGXVehicleContinuousTrackDevicePtr trackDevice = new AGXVehicleContinuousTrackDevice(desc);
    return loader.readDevice(trackDevice, node);
}

SceneDevice* createSceneAGXVehicleContinuousTrackDevice(Device* device)
{
    return new SceneTrackDevice(static_cast<AGXVehicleContinuousTrackDevice*>(device));
}


struct TypeRegistration
{
    TypeRegistration() {
        YAMLBodyLoader::addNodeType("AGXVehicleContinuousTrackDevice", readAGXVehicleContinuousTrackDevice);
        SceneDevice::registerSceneDeviceFactory<AGXVehicleContinuousTrackDevice>(createSceneAGXVehicleContinuousTrackDevice);
        if(AGXObjectFactory::checkModuleEnalbled("AGX-Vehicle") || AGXObjectFactory::checkModuleEnalbled("AgX-Vehicle")){
        }else {
            std::cout << "Please check you have AGX-Vehicle module license."
                "AGXVehicleContinuousTrackDevice should not work." << std::endl;
        }
    }
} registration;

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDeviceDesc& desc)
    : AGXVehicleContinuousTrackDeviceDesc(desc)
{
    initialize();
}

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char* AGXVehicleContinuousTrackDevice::typeName()
{
    return "AGXVehicleContinuousTrackDevice";
}


void AGXVehicleContinuousTrackDevice::copyStateFrom(const AGXVehicleContinuousTrackDevice& other)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    AGXVehicleContinuousTrackDevice& dev = const_cast<AGXVehicleContinuousTrackDevice&>(other);
    dev.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    m_trackStates = other.m_trackStates;
}


void AGXVehicleContinuousTrackDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AGXVehicleContinuousTrackDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXVehicleContinuousTrackDevice&>(other));
}


DeviceState* AGXVehicleContinuousTrackDevice::cloneState() const
{
    return new AGXVehicleContinuousTrackDevice(*this, true);
}


Referenced* AGXVehicleContinuousTrackDevice::doClone(CloneMap*) const
{
    return new AGXVehicleContinuousTrackDevice(*this);
}


void AGXVehicleContinuousTrackDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AGXVehicleContinuousTrackDevice))){
        Device::forEachActualType(func);
    }
}


int AGXVehicleContinuousTrackDevice::stateSize() const
{
    return numberOfNodes * 10;
}


const double* AGXVehicleContinuousTrackDevice::readState(const double* buf)
{
    if(m_trackStates.size() < numberOfNodes){
        m_trackStates.resize(numberOfNodes);
    }
    int i = 0;
    for(int j=0; j < numberOfNodes; ++j){
        TrackState& state = m_trackStates[j];
        state.boxSize << buf[i], buf[i+1], buf[i+2];
        i += 3;
        state.position.translation() << buf[i], buf[i+1], buf[i+2];
        i += 3;
        state.position.linear() = Quat(buf[i], buf[i+1], buf[i+2], buf[i+3]).toRotationMatrix();
        i += 4;
    }
    return buf + i;
}

double* AGXVehicleContinuousTrackDevice::writeState(double* out_buf) const
{
    double* out_buf0 = out_buf;
    
    int i = 0;
    int j = 0;

    while(j < m_trackStates.size()){
        const TrackState& state = m_trackStates[j++];
        const auto& s = state.boxSize;
        out_buf[i++] = s.x();
        out_buf[i++] = s.y();
        out_buf[i++] = s.z();
        const auto& T = state.position;
        auto p = T.translation();
        out_buf[i++] = p.x();
        out_buf[i++] = p.y();
        out_buf[i++] = p.z();
        Quat q(T.linear());
        out_buf[i++] = q.w();
        out_buf[i++] = q.x();
        out_buf[i++] = q.y();
        out_buf[i++] = q.z();
    }
    out_buf += i;

    int n = numberOfNodes - j;
    if(n > 0){
        int size = n * 10;
        std::fill(out_buf, out_buf + size, 0.0);
        out_buf += size;
    }

    return out_buf;
}

void AGXVehicleContinuousTrackDevice::initialize()
{
    m_trackStates.clear();
}

Vector3 AGXVehicleContinuousTrackDevice::getUpAxis() const
{
    return upAxis;
}

int AGXVehicleContinuousTrackDevice::getNumNodes() const
{
    return numberOfNodes;
}

const vector<string> AGXVehicleContinuousTrackDevice::getSprocketNames() const
{
    return sprocketNames;
}

const vector<string> AGXVehicleContinuousTrackDevice::getIdlerNames() const
{
    return idlerNames;
}

const vector<string> AGXVehicleContinuousTrackDevice::getRollerNames() const
{
    return rollerNames;
}

void AGXVehicleContinuousTrackDevice::setDesc(const AGXVehicleContinuousTrackDeviceDesc& desc){
    static_cast<AGXVehicleContinuousTrackDeviceDesc&>(*this) = desc;
}

void AGXVehicleContinuousTrackDevice::getDesc(AGXVehicleContinuousTrackDeviceDesc& desc)
{
    desc = static_cast<AGXVehicleContinuousTrackDeviceDesc&>(*this);
}

void AGXVehicleContinuousTrackDevice::reserveTrackStateSize(const unsigned int& num) {
    m_trackStates.reserve(num);
}

void AGXVehicleContinuousTrackDevice::addTrackState(const Vector3& boxSize, const Position& pos) {
    TrackState s;
    s.boxSize = boxSize;
    s.position = pos;
    m_trackStates.push_back(s);
}

TrackStates& AGXVehicleContinuousTrackDevice::getTrackStates() {
    return m_trackStates;
}

SgShape* AGXVehicleContinuousTrackDevice::getNodeShape(){
    return nodeShape;
}

}
