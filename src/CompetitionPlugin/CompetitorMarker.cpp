/**
   \file
   \author Shin'ichiro Nakaoka
*/

#include "CompetitorMarker.h"
#include <cnoid/StdBodyLoader>
#include <cnoid/StdBodyFileUtil>

using namespace std;
using namespace cnoid;


CompetitorMarker::CompetitorMarker()
{
    setMarkerType(MarkerDevice::SPHERE_MARKER);
    setMarkerSize(0.04);
    setColor(Vector3f(1.0f, 0.0f, 0.0f));
    setTransparency(0.0f);
}


const char* CompetitorMarker::typeName() const
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


namespace {

StdBodyFileDeviceTypeRegistration<CompetitorMarker>
registerMarkerDevice(
    "CompetitorMarker",
    [](StdBodyLoader* loader, const Mapping* info){
        CompetitorMarkerPtr marker = new CompetitorMarker;
        if(marker->readSpecifications(info)){
            return loader->readDevice(marker, info);
        }
        return false;
    },
    [](StdBodyWriter* /* writer */, Mapping* info, const CompetitorMarker* marker){
        return marker->writeSpecifications(info);
    });
}
