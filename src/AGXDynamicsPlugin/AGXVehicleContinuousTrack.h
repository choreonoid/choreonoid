#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H

#include "AGXBodyExtension.h"
#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/DeviceList>

namespace cnoid {

class AGXVehicleContinuousTrack : public AGXBodyExtension{
public:
    AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBody* const agxBody);
    void updateTrackState();
private:
    AGXVehicleContinuousTrackDevicePtr _device = nullptr;
    agxVehicle::TrackRef _track;
};



}


#endif