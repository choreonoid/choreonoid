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
        Link* l_link = l_agxLink->getOrgLink();
        const Vector3& a = l_link->attitude() * l_link->a();            // rotational axis
        const Vector3& u = l_link->attitude() * device->getUpAxis();    // up axis
        const agx::Vec3 ny = agx::Vec3(a(0), a(1), a(2)).normal();
        const agx::Vec3 nz = agx::Vec3(u(0), u(1), u(2)).normal();
        const agx::Vec3 nx = ny.cross(nz);
        agx::OrthoMatrix3x3 rotation;
        rotation.setColumn(0, nx);
        rotation.setColumn(1, ny);
        rotation.setColumn(2, nz);
        agx::Quat q;
        rotation.get(q);

        AGXVehicleTrackWheelDesc l_desc;
        l_desc.rigidbody = l_agxLink->getAGXRigidBody();
        l_desc.model = model;
        l_desc.radius = l_cylinder->getRadius();
        l_desc.rbRelTransform.setRotate(l_desc.rigidbody->getRotation());
        l_desc.rbRelTransform.setRotate(q * l_desc.rigidbody->getRotation());
        //return AGXObjectFactory::createVehicleTrackWheel(l_desc);

        agxVehicle::TrackWheelRef wheel = AGXObjectFactory::createVehicleTrackWheel(l_desc);
        agx::OrthoMatrix3x3 mat;
        l_agxLink->getAGXRigidBody()->getRotation().get(mat);
        std::cout << "rigid " <<  mat << std::endl;
        l_cylinder->getGeometry()->getRotation().get(mat);
        std::cout << "geometry " <<  mat << std::endl;
        std::cout << "rotation " << rotation << std::endl;
        l_desc.rbRelTransform.get(mat);
        std::cout << "rbRelTrans " << mat << std::endl;
        wheel->getTransform().get(mat);
        std::cout << "wheel " << mat << std::endl;
        return wheel;
    };

    auto createWheels = [createWheel, &trackDesc](const int& num, const string* names, const agxVehicle::TrackWheel::Model& model)
    {
        for(int i = 0; i < num; ++i){
            trackDesc.trackWheelRefs.push_back(createWheel(names[i], model));
        }
    };

    // Create sprocket, idler and roller wheels
    createWheels(device->numSprocketNames(), device->getSprocketNames(), agxVehicle::TrackWheel::Model::SPROCKET);
    createWheels(device->numIdlerNames(), device->getIdlerNames(), agxVehicle::TrackWheel::Model::IDLER);
    createWheels(device->numRollerNames(), device->getRollerNames(), agxVehicle::TrackWheel::Model::ROLLER);

    // Create track
    AGXVehicleContinuousTrackDeviceDesc desc;
    device->getDesc(desc);
    trackDesc.numberOfNodes = desc.numberOfNodes;
    trackDesc.nodeThickness = desc.nodeThickness;
    trackDesc.nodeWidth = desc.nodeWidth;
    trackDesc.nodeDistanceTension = desc.nodeDistanceTension;
    trackDesc.hingeCompliance = desc.hingeCompliance;
    trackDesc.stabilizingHingeFrictionParameter = desc.stabilizingHingeFrictionParameter;
    trackDesc.enableMerge = desc.enableMerge;
    trackDesc.numNodesPerMergeSegment = desc.numNodesPerMergeSegment;
    switch(desc.contactReductionLevel)
    {
        case 0:
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::NONE;
            break;
        case 1:
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::MINIMAL;
            break;
        case 2:
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::MODERATE;
            break;
        case 3:
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::AGRESSIVE;
            break;
        default:
            break;
    }
    _track = AGXObjectFactory::createVehicleTrack(trackDesc);
    addAGXAssembly(_track);
}


}