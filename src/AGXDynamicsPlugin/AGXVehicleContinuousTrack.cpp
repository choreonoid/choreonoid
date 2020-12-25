#include "AGXVehicleContinuousTrack.h"
#include "AGXBody.h"
#include "AGXScene.h"
#include <agx/version.h>


using namespace std;
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


AGXVehicleContinuousTrack::AGXVehicleContinuousTrack(AGXVehicleContinuousTrackDevice* const device, AGXBody* const agxBody) :
    AGXBodyExtension(agxBody)
{
    m_device = device;
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
        double radius = 0.0;
        if(l_agxLink->getAGXGeometry()->getShapes().size() <= 0 ) return nullptr;
        agxCollide::Shape* shape = l_agxLink->getAGXGeometry()->getShapes()[0];
        if(auto l_cylinder = dynamic_cast<agxCollide::Cylinder*>(shape)){
            radius = l_cylinder->getRadius();
        }else if(auto l_capsule = dynamic_cast<agxCollide::Capsule*>(shape)){
            radius = l_capsule->getRadius();
        }else{
            std::cout << "failed to cast agxCollide::Cylinder or agxCollide::Capsule." << std::endl;
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
        l_desc.radius = radius;
        l_desc.rbRelTransform.setRotate(l_desc.rigidbody->getRotation());
        l_desc.rbRelTransform.setRotate(rotation);
        return AGXObjectFactory::createVehicleTrackWheel(l_desc);
    };

    auto createWheels = [createWheel, &trackDesc](const vector<string>& names, const agxVehicle::TrackWheel::Model& model)
    {
        for(auto it : names){
            if(auto wheel = createWheel(it, model)){
                trackDesc.trackWheelRefs.push_back(wheel);
            }
        }
    };

    // Create sprocket, idler and roller wheels
    createWheels(device->getSprocketNames(), agxVehicle::TrackWheel::Model::SPROCKET);
    createWheels(device->getIdlerNames(), agxVehicle::TrackWheel::Model::IDLER);
    createWheels(device->getRollerNames(), agxVehicle::TrackWheel::Model::ROLLER);

    // Create track
    AGXVehicleContinuousTrackDeviceDesc desc;
    device->getDesc(desc);
    trackDesc.numberOfNodes = (agx::UInt)desc.numberOfNodes;
    trackDesc.nodeThickness = desc.nodeThickness;
    trackDesc.nodeWidth = desc.nodeWidth;
    trackDesc.nodeDistanceTension = desc.nodeDistanceTension;
    trackDesc.nodeThickerThickness = desc.nodeThickerThickness;
    trackDesc.useThickerNodeEvery = desc.useThickerNodeEvery;
    trackDesc.hingeCompliance = desc.hingeCompliance;
    trackDesc.hingeSpookDamping= desc.hingeSpookDamping;
    trackDesc.minStabilizingHingeNormalForce = desc.minStabilizingHingeNormalForce;
    trackDesc.stabilizingHingeFrictionParameter = desc.stabilizingHingeFrictionParameter;
    trackDesc.nodesToWheelsMergeThreshold = desc.nodesToWheelsMergeThreshold;
    trackDesc.nodesToWheelsSplitThreshold = desc.nodesToWheelsSplitThreshold;
    trackDesc.enableMerge = desc.enableMerge;
    trackDesc.numNodesPerMergeSegment = (agx::UInt)desc.numNodesPerMergeSegment;
    switch(desc.contactReduction)
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
#if AGX_VERSION_GREATER_OR_EQUAL(2 ,21, 1, 0)
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::AGGRESSIVE;
#else
            trackDesc.contactReduction =  agxVehicle::TrackInternalMergeProperties::ContactReduction::AGRESSIVE;
#endif
            break;
        default:
            break;
    }
    trackDesc.enableLockToReachMergeCondition = desc.enableLockToReachMergeCondition;
    trackDesc.lockToReachMergeConditionCompliance = desc.lockToReachMergeConditionCompliance;
    trackDesc.lockToReachMergeConditionSpookDamping = desc.lockToReachMergeConditionSpookDamping;
    trackDesc.maxAngleMergeCondition = desc.maxAngleMergeCondition;
    m_track = AGXObjectFactory::createVehicleTrack(trackDesc);
    if(!m_track) return;
    AGXLink* agxLink = agxBody->getAGXLink(m_device->link()->name());
    m_track->addGroup(agxLink->getCollisionGroupName());

    AGXScene* agxScene = getAGXBody()->getAGXScene();
    // Set material
    m_track->setMaterial(agxScene->getMaterial(desc.materialName));

    // Add to simulation
    agxScene->getSimulation()->add((agxSDK::Assembly*)m_track);
    agxScene->getSimulation()->add(new TrackListener(this));

    /* Set collision Group*/
    // BodyCollisionGroup(bodies, wheels, guides)
    // trackCollisionGroup(tracks)
    // disableTrackCollisionGroup(bodies)
    std::stringstream trackCollision, disableTrackCollision;
    trackCollision << "trackCollision" << agx::UuidGenerator().generate().str() << std::endl;
    disableTrackCollision << "trackCollision" << agx::UuidGenerator().generate().str() << std::endl;
    m_track->addGroup(trackCollision.str());
    getAGXBody()->addCollisionGroupNameToAllLink(disableTrackCollision.str());
    getAGXBody()->getAGXScene()->setCollisionPair(trackCollision.str(), disableTrackCollision.str(), false);
    for(auto wheel : trackDesc.trackWheelRefs){
        agxCollide::GeometryRef geometry = wheel->getRigidBody()->getGeometries().front();
        geometry->removeGroup(disableTrackCollision.str());
    }
    for(auto guide : desc.guideNames){
        if(agx::RigidBody* rigid = getAGXBody()->getAGXRigidBody(guide)){
            if(agxCollide::GeometryRef geometry = rigid->getGeometries().front()){
                geometry->removeGroup(disableTrackCollision.str());
            }
        }
    }

    /* Rendering */
    // Retrieve size and transform of node from track for graphic rendering
    // Limit only one geometry and one shape
    m_device->initialize();
    m_device->reserveTrackStateSize((unsigned int)m_track->nodes().size());
    for(auto node : m_track->nodes()){
        if(agxCollide::Shape* const shape = node->getRigidBody()
                ->getGeometries().front()->getShapes().front()){
            if(agxCollide::Box* const box = static_cast<agxCollide::Box*>(shape)){
                const agx::Vec3& e = box->getHalfExtents() * 2.0;
                Vector3 size(e[0], e[1], e[2]);
                Isometry3 pos;
                convertToIsometry3(node->getRigidBody()->getTransform(), pos);
                m_device->addTrackState(size, pos);
            }
        }
    }
    m_device->notifyStateChange();
    //printParameters(trackDesc);
}

void AGXVehicleContinuousTrack::updateTrackState() {
    for(size_t i = 0; i < m_track->nodes().size(); ++i){
        convertToIsometry3(
            m_track->nodes()[i]->getRigidBody()->getGeometries().front()->getTransform(),
            m_device->getTrackStates()[i].position);        
    }
    m_device->notifyStateChange();
}

#define PRINT_PARAMETER(FIELD1)  std::cout << #FIELD1 << " " << FIELD1 << std::endl
void AGXVehicleContinuousTrack::printParameters(const AGXVehicleTrackDesc& desc){
    cout << m_device->link()->name() << endl;
    PRINT_PARAMETER(desc.numberOfNodes);
    PRINT_PARAMETER(desc.nodeThickness);
    PRINT_PARAMETER(desc.nodeWidth);
    PRINT_PARAMETER(desc.nodeThickerThickness);
    PRINT_PARAMETER(desc.useThickerNodeEvery);
    PRINT_PARAMETER(desc.hingeCompliance);
    PRINT_PARAMETER(desc.hingeSpookDamping);
    PRINT_PARAMETER(desc.minStabilizingHingeNormalForce);
    PRINT_PARAMETER(desc.stabilizingHingeFrictionParameter);
    PRINT_PARAMETER(desc.enableMerge);
    PRINT_PARAMETER(desc.numNodesPerMergeSegment);
    PRINT_PARAMETER(desc.contactReduction);
    PRINT_PARAMETER(desc.enableLockToReachMergeCondition);
    PRINT_PARAMETER(desc.lockToReachMergeConditionCompliance);
    PRINT_PARAMETER(desc.lockToReachMergeConditionSpookDamping);
    PRINT_PARAMETER(desc.maxAngleMergeCondition);
    cout << m_track->getInternalMergeProperties()->getEnableMerge() << endl;
    cout << m_track->getInternalMergeProperties()->getEnableLockToReachMergeCondition() << endl;

}
#undef PRINT_PARAMETER



}
