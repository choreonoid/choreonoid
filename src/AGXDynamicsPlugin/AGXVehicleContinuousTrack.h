#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_CONTINOUS_TRACK_H

#include "AGXBodyExtension.h"
#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/DeviceList>

namespace cnoid {

class AGXVehicleContinuousTrack : public AGXBodyExtension{
private:
    AGXVehicleContinuousTrackDevicePtr m_device = nullptr;
    agxVehicle::TrackRef m_track;
public:
    AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBody* const agxBody);
    void updateTrackState();
private:
    void printParameters(const AGXVehicleTrackDesc& desc);

};



}


#endif