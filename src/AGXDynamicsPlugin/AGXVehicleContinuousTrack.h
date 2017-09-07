#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H

#include "AGXBodyPart.h"
#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/DeviceList>

namespace cnoid {

class AGXVehicleContinuousTrack : public AGXBodyPart{
public:
    AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, const AGXBodyPtr agxBody);
private:
    AGXVehicleContinuousTrackDevicePtr _device = nullptr;
    agxVehicle::TrackRef _track;
};



}


#endif