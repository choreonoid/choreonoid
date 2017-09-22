#include "AGXVehicleContinuousTrack.h"
#include "AGXBody.h"
#include "AGXScene.h"
#include <iostream>

namespace cnoid {

class TrackListener : public agxSDK::StepEventListener
{
public:
    TrackListener( AGXVehicleContinuousTrack* track )
            : m_track( track )
    {
        setMask( POST_STEP );
    }

    virtual void post( const agx::TimeStamp& /*t*/ )
    {
        m_track->updateTrackState();
    }

private:
    AGXVehicleContinuousTrack*  m_track;
};


AGXVehicleContinuousTrack::AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBody* const agxBody) : AGXBodyExtension(agxBody)
{
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
        const Vector3& a = l_link->a();            // rotational axis
        const Vector3& u = device->getUpAxis();    // up axis
        const agx::Vec3 ny = agx::Vec3(a(0), a(1), a(2)).normal();
        const agx::Vec3 nz = agx::Vec3(u(0), u(1), u(2)).normal();
        if(ny * nz > 1e-6) return nullptr;   // Vectors maybe not orthogonal to each other
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
        l_desc.rbRelTransform.setRotate(rotation);
        //l_desc.rbRelTransform.setRotate(q * l_desc.rigidbody->getRotation());
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
    getAssembly()->add(_track);
    getAGXBody()->getAGXScene()->add(getAssembly());
    getAGXBody()->getAGXScene()->getSimulation()->add(new TrackListener(this));

    // Retrieve size and transform of node from track for graphic rendering
    // Limit only one geometry and one shape
    _device->reserveTrackStateSize(_track->nodes().size());
    for(auto node : _track->nodes()){
        if(agxCollide::Shape* const shape = node->getRigidBody()
                ->getGeometries().front()->getShapes().front()){
            if(agxCollide::Box* const box = static_cast<agxCollide::Box*>(shape)){
                const agx::Vec3& e = box->getHalfExtents() * 2.0;
                Vector3 s(e[0], e[1], e[2]);
                const Affine3& transform = convertToAffine3(node->getRigidBody()->getTransform());
                _device->addTrackState(s, transform);
                std::cout << _device->getTrackStates().back().transform.translation() << std::endl;
            }
        }
    }
    _device->notifyStateChange();
}

void AGXVehicleContinuousTrack::updateTrackState() {
    for(size_t i = 0; i < _track->nodes().size(); ++i){
        _device->getTrackStates()[i].transform =
                convertToAffine3(_track->nodes()[i]->getRigidBody()->getTransform());
    }
    _device->notifyStateChange();
}


}