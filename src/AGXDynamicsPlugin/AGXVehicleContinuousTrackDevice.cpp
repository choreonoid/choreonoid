#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/YAMLBodyLoader>
#include <cnoid/YAMLReader>
#include <cnoid/SceneDevice>
#include "AGXObjectFactory.h"

using namespace std;

namespace cnoid {

bool readAGXVehicleContinuousTrackDevice(YAMLBodyLoader& loader, Mapping& node)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    if(!node.read("numberOfNodes", desc.numberOfNodes)) return false;
    if(!node.read("nodeThickness", desc.nodeThickness)) return false;
    if(!node.read("nodeWidth", desc.nodeWidth)) return false;
    if(!node.read("nodeDistanceTension", desc.nodeDistanceTension)) return false;

    // Get name of wheels from yaml 
    const auto packToVectorString = [](ValueNodePtr const vnptr, vector<string>& vs) ->bool
    {
        if(!vnptr) return false;
        Listing& list = *vnptr->toListing();
        for(int i=0; i < list.size(); i++){
            vs.push_back(list[i].toString());
        }
        return true;
    };
    MappingPtr info = static_cast<Mapping*>(node.clone());
    packToVectorString(info->extract("sprocketNames"), desc.sprocketNames);
    packToVectorString(info->extract("idlerNames"), desc.idlerNames);
    packToVectorString(info->extract("rollerNames"), desc.rollerNames);
    Listing& u = *info->extract("upAxis")->toListing();
    if(!u.size() == 3) return false;
    desc.upAxis = Vector3(u[0].toDouble(), u[1].toDouble(), u[2].toDouble());

    AGXVehicleContinuousTrackDevicePtr trackDevice = new AGXVehicleContinuousTrackDevice(desc);
    return loader.readDevice(trackDevice, node);
}

SceneDevice* createSceneAGXVehicleContinuousTrackDevice(Device* device)
{
//    auto sceneSmoke = new SceneSmoke;
    //auto sceneDevice = new SceneDevice(device, sceneSmoke);

    //sceneDevice->setFunctionOnTimeChanged(
    //    [sceneSmoke](double time){
    //        sceneSmoke->setTime(time);
    //        sceneSmoke->notifyUpdate();
    //    });
    //        
    //return sceneDevice;
    return nullptr;
}
                        
struct TypeRegistration
{
    TypeRegistration() {
        if(AGXObjectFactory::checkModuleEnalbled("AGX-Vehicle")){
            YAMLBodyLoader::addNodeType("AGXVehicleContinuousTrackDevice", readAGXVehicleContinuousTrackDevice);
            SceneDevice::registerSceneDeviceFactory<AGXVehicleContinuousTrackDevice>(createSceneAGXVehicleContinuousTrackDevice);
            std::cout << "AGX-Vehicle is enabled." << std::endl;
        }else {
            std::cout << "Please check you have AGX-Vehicle module license." << std::endl;
        }
    }
} registration;

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDeviceDesc& desc)
{
    upAxis = desc.upAxis;
    numberOfNodes = desc.numberOfNodes;
    nodeThickness = desc.nodeThickness;
    nodeWidth = desc.nodeWidth;
    nodeDistanceTension = desc.nodeDistanceTension;
    sprocketNames = desc.sprocketNames;
    idlerNames = desc.idlerNames;
    rollerNames = desc.rollerNames;
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
    upAxis = other.upAxis;
    numberOfNodes = other.numberOfNodes;
    nodeThickness = other.nodeThickness;
    nodeWidth = other.nodeWidth;
    nodeDistanceTension = other.nodeDistanceTension;
    sprocketNames = other.sprocketNames;
    idlerNames = other.idlerNames;
    rollerNames = other.rollerNames;
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

int AGXVehicleContinuousTrackDevice::getNumberOfNodes() const
{
    return numberOfNodes;
}

double AGXVehicleContinuousTrackDevice::getNodeThickness() const
{
    return nodeThickness;
}

double AGXVehicleContinuousTrackDevice::getNodeWidth() const
{
    return nodeWidth;
}

double AGXVehicleContinuousTrackDevice::getNodeDistanceTension() const
{
    return nodeDistanceTension;
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
    if(rollerNames.empty()) return nullptr;
    return &rollerNames.front();
}


}