#include "AGXVehicleContinuousTrack.h"
#include "AGXBody.h"

namespace cnoid {

AGXVehicleContinuousTrack::AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBodyPtr agxBody){
    _device = device;
    AGXVehicleTrackDesc trackDesc;

    auto createWheel = [device, agxBody](const string& name,
        const agxVehicle::TrackWheel::Model model) -> agxVehicle::TrackWheelRef
    {
        AGXLinkPtr l_agxLink = agxBody->getAGXLink(name);
        if(!l_agxLink) return nullptr;
        agx::RigidBody* l_rigid = l_agxLink->getAGXRigidBody();
        agxCollide::CylinderRef l_cylinder = dynamic_cast<agxCollide::Cylinder*>(l_agxLink->getAGXGeometry()->getShape());
        AGXVehicleTrackWheelDesc l_desc;
        l_desc.rigidbody = l_rigid;
        l_desc.model = model;
        l_desc.radius = l_cylinder->getRadius();
       return AGXObjectFactory::createVehicleTrackWheel(l_desc);
    };

    auto createWheels = [createWheel, &trackDesc](const int& num, const string* names, const agxVehicle::TrackWheel::Model& model)
    {
        for(int i = 0; i < num; ++i){
            agxVehicle::TrackWheelRef wheel;
            wheel = createWheel(names[i], model);
            trackDesc.trackWheelRefs.push_back(wheel);
        }
    };

    // Create sprocket wheels
    createWheels(device->numSprocketNames(), device->getSprocketNames(), agxVehicle::TrackWheel::Model::SPROCKET);
    // Create idler wheels
    createWheels(device->numIdlerNames(), device->getIdlerNames(), agxVehicle::TrackWheel::Model::IDLER);
    // Create roller wheels
    createWheels(device->numRollerNames(), device->getRollerNames(), agxVehicle::TrackWheel::Model::ROLLER);

    // Create track
    trackDesc.numberOfNodes = device->getNumberOfNodes();
    trackDesc.nodeThickness = device->getNodeThickness();
    trackDesc.nodeWidth = device->getNodeWidth();
    trackDesc.nodeDistanceTension = device->getNodeDistanceTension();
    _track = AGXObjectFactory::createVehicleTrack(trackDesc);
    addAGXAssembly(_track);
}


}