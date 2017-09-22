#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include "AGXObjectFactory.h"
#include <cnoid/Link>

using namespace std;

namespace cnoid {

class SceneTrackDevice : public SceneDevice
{
    AGXVehicleContinuousTrackDevice* trackDevice;

public:
    SceneTrackDevice(AGXVehicleContinuousTrackDevice* device)
        : SceneDevice(device)
    {
        trackDevice = device;
        MeshGenerator meshGenerator;
        for (int i = 0; i < trackDevice->getNumNodes(); ++i) {
            auto position = new SgPosTransform;
            addChild(position, true);
        }
        setFunctionOnStateChanged([&](){ updateTrack(); });
    }

    void updateTrack() {
        MeshGenerator meshGenerator;
//                const int numShoes = 1;
//        for(int i=0; i < numShoes; ++i){
//            auto position = new SgPosTransform;
//            auto shape = new SgShape;
//            //shape->setMesh(meshGenerator.generateBox(Vector3(0.6, 0.09, 0.3)));
//            shape->setMesh(meshGenerator.generateBox(Vector3(5, 5, 5)));
//            position->addChild(shape);
//            addChild(position, true);
//        }

        if (trackDevice->getTrackStates().size() <= 0) return;
        const Position &linkTransform_inv = trackDevice->link()->T().inverse();
        for (int i = 0; i < trackDevice->getTrackStates().size(); ++i) {
            SgPosTransform* t = static_cast<SgPosTransform*>(child(i));
            t->setTransform(linkTransform_inv * trackDevice->getTrackStates()[i].transform);
            if(t->numChildren() > 0) continue;
            auto shape = new SgShape;
            shape->setMesh(meshGenerator.generateBox(trackDevice->getTrackStates()[i].boxSize));
            t->addChild(shape, true);
        }
    }
};


bool readAGXVehicleContinuousTrackDevice(YAMLBodyLoader& loader, Mapping& node)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    if(!node.read("numberOfNodes", desc.numberOfNodes)) return false;
    if(!node.read("nodeThickness", desc.nodeThickness)) return false;
    if(!node.read("nodeWidth", desc.nodeWidth)) return false;
    if(!node.read("nodeDistanceTension", desc.nodeDistanceTension)) return false;
    node.read("hingeCompliance", desc.hingeCompliance);
    node.read("stabilizingHingeFrictionParameter", desc.stabilizingHingeFrictionParameter);
    node.read("enableMerge", desc.enableMerge);
    node.read("numNodesPerMergeSegment", desc.numNodesPerMergeSegment);
    node.read("lockToReachMergeConditionCompliance", desc.lockToReachMergeConditionCompliance);
    node.read("contactReductionLevel", desc.contactReductionLevel);

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
    ValueNodePtr const upAxis = info->extract("upAxis");
    if(upAxis){
        Listing& u = *upAxis->toListing();
        if(u.size() != 3) return false;
        desc.upAxis = Vector3(u[0].toDouble(), u[1].toDouble(), u[2].toDouble());
    }else{
        return false;
    }
    std::cout << desc.sprocketNames.size() << std::endl;
    AGXVehicleContinuousTrackDevicePtr trackDevice = new AGXVehicleContinuousTrackDevice(desc);
    std::cout << trackDevice->getSprocketNames()->size() << std::endl;
    return loader.readDevice(trackDevice, node);
}

SceneDevice* createSceneAGXVehicleContinuousTrackDevice(Device* device)
{
    return new SceneTrackDevice(static_cast<AGXVehicleContinuousTrackDevice*>(device));
}


struct TypeRegistration
{
    TypeRegistration() {
        if(AGXObjectFactory::checkModuleEnalbled("AGX-Vehicle") || AGXObjectFactory::checkModuleEnalbled("AgX-Vehicle")){
            YAMLBodyLoader::addNodeType("AGXVehicleContinuousTrackDevice", readAGXVehicleContinuousTrackDevice);
            SceneDevice::registerSceneDeviceFactory<AGXVehicleContinuousTrackDevice>(createSceneAGXVehicleContinuousTrackDevice);
            std::cout << "AGX-Vehicle is enabled." << std::endl;
        }else {
            std::cout << "Please check you have AGX-Vehicle module license." << std::endl;
        }
    }
} registration;

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDeviceDesc& desc)
    : AGXVehicleContinuousTrackDeviceDesc(desc)
{
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
    on_ = other.on_;
    AGXVehicleContinuousTrackDeviceDesc desc;
    AGXVehicleContinuousTrackDevice& dev = const_cast<AGXVehicleContinuousTrackDevice&>(other);
    dev.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    _trackStates = other._trackStates;
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
    return new AGXVehicleContinuousTrackDevice(*this, false);
}


Device* AGXVehicleContinuousTrackDevice::clone() const
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
    return 1;
}


const double* AGXVehicleContinuousTrackDevice::readState(const double* buf)
{
    on_ = buf[0];
    return buf + 1;
}


double* AGXVehicleContinuousTrackDevice::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    return out_buf + 1;
}


Vector3 AGXVehicleContinuousTrackDevice::getUpAxis() const
{
    return upAxis;
}

int AGXVehicleContinuousTrackDevice::getNumNodes() const
{
    return numberOfNodes;
}

int AGXVehicleContinuousTrackDevice::numSprocketNames() const
{
    return sprocketNames.size();
}

const string* AGXVehicleContinuousTrackDevice::getSprocketNames() const
{
    if(sprocketNames.empty()) return nullptr;
    return &sprocketNames.front();
}

int AGXVehicleContinuousTrackDevice::numIdlerNames() const
{
    return idlerNames.size();
}

const string* AGXVehicleContinuousTrackDevice::getIdlerNames() const
{
    if(idlerNames.empty()) return nullptr;
    return &idlerNames.front();
}

int AGXVehicleContinuousTrackDevice::numRollerNames() const
{
    return rollerNames.size();;
}

const string* AGXVehicleContinuousTrackDevice::getRollerNames() const
{
    if (rollerNames.empty()) return nullptr;
    return &rollerNames.front();
}

void AGXVehicleContinuousTrackDevice::setDesc(const AGXVehicleContinuousTrackDeviceDesc& desc){
    static_cast<AGXVehicleContinuousTrackDeviceDesc&>(*this) = desc;
}

void AGXVehicleContinuousTrackDevice::getDesc(AGXVehicleContinuousTrackDeviceDesc& desc)
{
    desc = static_cast<AGXVehicleContinuousTrackDeviceDesc&>(*this);
}

void AGXVehicleContinuousTrackDevice::reserveTrackStateSize(const unsigned int& num) {
    _trackStates.reserve(num);
}

void AGXVehicleContinuousTrackDevice::addTrackState(const Vector3& boxSize, const Position& transform) {
    TrackState s;
    s.boxSize = boxSize;
    s.transform = transform;
    _trackStates.push_back(s);
}

TrackStates& AGXVehicleContinuousTrackDevice::getTrackStates() {
    return _trackStates;
}


}