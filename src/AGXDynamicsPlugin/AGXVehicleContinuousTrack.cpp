#include "AGXVehicleContinuousTrack.h"
#include "AGXBody.h"

namespace cnoid {

AGXVehicleContinuousTrack::AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBodyPtr agxBody){
    _device = device;
    AGXVehicleTrackDesc trackDesc;

    auto createWheel = [device, agxBody](const string& name,
        const agxVehicle::TrackWheel::Model model) -> agxVehicle::TrackWheelRef
    {
        // get agxlink
        AGXLinkPtr l_agxLink = agxBody->getAGXLink(name);
        if(!l_agxLink){
            std::cout << "failed to get agxlink by linkname at AGXVehicleCotinuousTrack." << std::endl;
            return nullptr;
        }
        // get raduis of wheel
        if(l_agxLink->getAGXGeometry()->getShapes().size() <= 0 ) return nullptr;
        agxCollide::Shape* shape = l_agxLink->getAGXGeometry()->getShapes()[0];
        agxCollide::Cylinder* l_cylinder = dynamic_cast<agxCollide::Cylinder*>(shape);
        if(!l_cylinder){
            std::cout << "failed to cast agxCollide::Cylinder." << std::endl;
            return nullptr;
        }
        // create rotate matrix
        const Vector3& a = l_agxLink->getOrgLink()->a(); // rotational axis
        const Vector3& u = device->getUpAxis();          // up axis
        const agx::Vec3 ny = agx::Vec3(a(0), a(1), a(2)).normal();
        const agx::Vec3 nz = agx::Vec3(u(0), u(1), u(2)).normal();
        agx::Vec3 nx = ny.cross(nz);
        agx::OrthoMatrix3x3 rotation;
        rotation.setColumn(0, nx);
        rotation.setColumn(1, ny);
        rotation.setColumn(2, nz);
        std::cout << "rotation " << rotation << std::endl;
        agx::OrthoMatrix3x3 mat;
        l_agxLink->getAGXRigidBody()->getRotation().get(mat);
        std::cout << "rigid " <<  mat << std::endl;
        std::cout << "geometry " << l_cylinder->getGeometry()->getRotation() << std::endl;

        AGXVehicleTrackWheelDesc l_desc;
        l_desc.rigidbody = l_agxLink->getAGXRigidBody();
        l_desc.model = model;
        l_desc.radius = l_cylinder->getRadius();
        //l_desc.rbRelTransform.setRotate(rotation);
        l_desc.rbRelTransform.setRotate(agx::Quat(-agx::PI_2, agx::Vec3::X_AXIS()));

       return AGXObjectFactory::createVehicleTrackWheel(l_desc);
    };

    auto createWheels = [createWheel, &trackDesc](const int& num, const string* names, const agxVehicle::TrackWheel::Model& model)
    {
        for(int i = 0; i < num; ++i){
            trackDesc.trackWheelRefs.push_back(createWheel(names[i], model));
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