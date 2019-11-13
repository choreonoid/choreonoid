/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "CompetitorMarker.h"
#include <cnoid/YAMLBodyLoader>

using namespace std;
using namespace cnoid;

namespace {

YAMLBodyLoader::NodeTypeRegistration
registerCompetitorMarker(
    "CompetitorMarker",
    [](YAMLBodyLoader& loader, Mapping& node){
        CompetitorMarkerPtr device = new CompetitorMarker;
        return device->readDescription(loader, node);
    });

}


CompetitorMarker::CompetitorMarker()
{
    setMarkerType(MarkerDevice::SPHERE_MARKER);
    setMarkerSize(0.04);
    setColor(Vector3f(1.0f, 0.0f, 0.0f));
    setTransparency(0.0f);
}


const char* CompetitorMarker::typeName()
{
    return "CompetitorMarker";
}


void CompetitorMarker::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(CompetitorMarker)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyMarkerDeviceStateFrom(static_cast<const MarkerDevice&>(other));
}


CompetitorMarker::CompetitorMarker(const CompetitorMarker& org, bool copyStateOnly)
    : MarkerDevice(org, copyStateOnly)
{

}


DeviceState* CompetitorMarker::cloneState() const
{
    return new CompetitorMarker(*this, true);
}


Referenced* CompetitorMarker::doClone(CloneMap*) const
{
    return new CompetitorMarker(*this);
}


void CompetitorMarker::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(CompetitorMarker))){
        MarkerDevice::forEachActualType(func);
    }
}
