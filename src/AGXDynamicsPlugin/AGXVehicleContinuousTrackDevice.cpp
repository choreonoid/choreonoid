#include "AGXVehicleContinuousTrackDevice.h"
#include <cnoid/StdBodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/YAMLReader>
#include <cnoid/StdSceneReader>
#include <cnoid/StdSceneWriter>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/EigenArchive>
#include "AGXObjectFactory.h"
#include <cnoid/Link>

using namespace std;

namespace cnoid {

class SceneTrackDevice : public SceneDevice
{
    AGXVehicleContinuousTrackDevice* m_trackDevice;
    std::vector<SgPosTransformPtr> m_sgTracks;
    SgSwitchableGroupPtr trackSwitch;
    SgUpdate sgUpdate;

public:
    SceneTrackDevice(AGXVehicleContinuousTrackDevice* device)
        : SceneDevice(device)
    {
        m_trackDevice = device;
        trackSwitch = new SgSwitchableGroup;
        trackSwitch->setTurnedOn(false);
        addChild(trackSwitch);
        setFunctionOnStateChanged([&](){ update(); });
        initialize();
    }

    void initialize(){
        m_sgTracks.clear();
        m_sgTracks.reserve(m_trackDevice->getNumNodes());
        for(int i = 0; i < m_trackDevice->getNumNodes(); ++i){
            SgPosTransformPtr sgTrack = new SgPosTransform;
            trackSwitch->addChild(sgTrack);
            m_sgTracks.push_back(sgTrack);
        }
    }

    void update(){
        if(m_trackDevice->getTrackStates().size() <= 0){
            trackSwitch->setTurnedOn(false, sgUpdate);
        } else {
            if(!trackSwitch->isTurnedOn()){
                createTrackShapes();
                trackSwitch->setTurnedOn(true, sgUpdate);
            }
            updateTrackPosition();
        }
    }

    void createTrackShapes(){
        MeshGenerator meshGenerator;
        const TrackStates& states = m_trackDevice->getTrackStates();
        for(size_t i = 0; i < states.size(); ++i) {
            m_sgTracks[i]->clearChildren();
            const auto& size = states[i].boxSize;
            if(size != Vector3::Zero()){
                auto shape = m_trackDevice->getNodeShape();
                if(shape){
                    shape = dynamic_cast<SgShape*>(shape->clone());
                } else {
                    shape = new SgShape;
                }
                shape->setMesh(meshGenerator.generateBox(size));
                m_sgTracks[i]->addChild(shape, sgUpdate);
            }
        }
    }

    void updateTrackPosition() {
        const Isometry3 &linkPos_inv = m_trackDevice->link()->T().inverse();
        const TrackStates& states = m_trackDevice->getTrackStates();
        for(size_t i = 0; i < states.size(); ++i){
            const auto& state = states[i];
            if(state.boxSize.x() == 0.0){
                trackSwitch->setTurnedOn(false, sgUpdate);
                break;
            } else {
                m_sgTracks[i]->setTransform(linkPos_inv * state.position);
            }
        }
        notifyUpdate(sgUpdate);
    }
};

#define NODE_READ(FIELD1) info->read(#FIELD1, desc.FIELD1)
bool readAGXVehicleContinuousTrackDevice(StdBodyLoader* loader, const Mapping* info)
{
    AGXVehicleContinuousTrackDeviceDesc desc;

    info->read({ "number_of_nodes", "numberOfNodes" }, desc.numberOfNodes);
    info->read({ "node_thickness", "nodeThickness" }, desc.nodeThickness);
    if(!info->read({ "node_width", "nodeWidth" }, desc.nodeWidth)){
        return false;
    }
    info->read({ "node_distance_tension", "nodeDistanceTension" }, desc.nodeDistanceTension);
    info->read({ "node_thicker_thickness", "nodeThickerThickness" }, desc.nodeThickerThickness);
    info->read({ "use_thicker_node_every", "useThickerNodeEvery" }, desc.useThickerNodeEvery);
    info->read({ "hinge_compliance", "hingeCompliance" }, desc.hingeCompliance);
    info->read({ "hinge_spook_damping", "hingeSpookDamping" }, desc.hingeSpookDamping);
    info->read({ "min_stabilizing_hinge_normal_force", "minStabilizingHingeNormalForce" },
               desc.minStabilizingHingeNormalForce);
    info->read({ "stabilizing_hinge_friction_parameter", "stabilizingHingeFrictionParameter" },
               desc.stabilizingHingeFrictionParameter);
    info->read({ "nodes_to_wheels_merge_treshold", "nodesToWheelsMergeThreshold" },
               desc.nodesToWheelsMergeThreshold);
    info->read({ "nodes_to_wheels_split_threshold", "nodesToWheelsSplitThreshold" },
               desc.nodesToWheelsSplitThreshold);
    info->read({ "enable_merge", "enableMerge" }, desc.enableMerge);
    info->read({ "num_nodes_per_merge_segment", "numNodesPerMergeSegment" }, desc.numNodesPerMergeSegment);
    info->read({ "contact_reduction", "contactReduction" }, desc.contactReduction);
    info->read({ "enable_lock_to_reach_merge_condition", "enableLockToReachMergeCondition" },
               desc.enableLockToReachMergeCondition);
    info->read({ "lock_to_reach_merge_condition_compliance", "lockToReachMergeConditionCompliance" },
               desc.lockToReachMergeConditionCompliance);
    info->read({ "lock_to_reach_merge_condition_spook_damping", "lockToReachMergeConditionSpookDamping" },
               desc.lockToReachMergeConditionSpookDamping);
    info->read({ "max_angle_merge_condition" "maxAngleMergeCondition" }, desc.maxAngleMergeCondition);
    info->read("material", desc.materialName);

    desc.nodeShape = dynamic_cast<SgShape*>(       
        loader->sceneReader()->readNode(info->findMapping("nodeShape"), "Shape"));

    // Get name of wheels from yaml
    const auto toVectorString =
        [](Listing* listing, vector<string>& vs) -> bool
        {
            if(!listing->isValid()){
                return false;
            }
            for(auto& value : *listing){
                vs.push_back(value->toString());
            }
            return true;
        };

    toVectorString(info->findListing({ "sprocket_names", "sprocketNames" }), desc.sprocketNames);
    toVectorString(info->findListing({ "idler_names", "idlerNames" }), desc.idlerNames);
    toVectorString(info->findListing({ "roller_names", "rollerNames" }), desc.rollerNames);
    toVectorString(info->findListing({ "guide_names", "guideNames" }), desc.guideNames);

    if(!read(info, { "up_axis", "upAxis" }, desc.upAxis)){
        return false;
    }

    AGXVehicleContinuousTrackDevicePtr trackDevice = new AGXVehicleContinuousTrackDevice(desc);
    return loader->readDevice(trackDevice, info);
}

bool writeAGXVehicleContinuousTrackDevice
(StdBodyWriter* writer, Mapping* info, const AGXVehicleContinuousTrackDevice* device)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    device->getDesc(desc);
    
    info->write("number_of_nodes", desc.numberOfNodes);
    info->write("node_thickness", desc.nodeThickness);
    info->write("node_width", desc.nodeWidth);
    info->write("node_distance_tension", desc.nodeDistanceTension);
    info->write("node_thicker_thickness", desc.nodeThickerThickness);
    info->write("use_thicker_node_every", desc.useThickerNodeEvery);
    info->write("hinge_compliance", desc.hingeCompliance);
    info->write("hinge_spook_damping", desc.hingeSpookDamping);
    info->write("min_stabilizing_hinge_normal_force", desc.minStabilizingHingeNormalForce);
    info->write("stabilizing_hinge_friction_parameter", desc.stabilizingHingeFrictionParameter);
    info->write("nodes_to_wheels_merge_threshold", desc.nodesToWheelsMergeThreshold);
    info->write("nodes_to_wheels_split_threshold", desc.nodesToWheelsSplitThreshold);
    info->write("enable_merge", desc.enableMerge);
    info->write("num_nodes_per_merge_segment", desc.numNodesPerMergeSegment);
    info->write("contact_reduction", desc.contactReduction);
    info->write("enable_lock_to_reach_merge_condition", desc.enableLockToReachMergeCondition);
    info->write("lock_to_reach_merge_condition_compliance", desc.lockToReachMergeConditionCompliance);
    info->write("lock_to_reach_merge_condition_spook_damping", desc.lockToReachMergeConditionSpookDamping);
    info->write("max_angle_merge_condition", desc.maxAngleMergeCondition);
    info->write("material", desc.materialName);

    if(auto shape = writer->sceneWriter()->writeScene(desc.nodeShape)){
        shape->remove("type");
        info->insert("nodeShape", shape);
    }

    info->writeAsListing("sprocket_names", desc.sprocketNames);
    info->writeAsListing("idler_names", desc.idlerNames);
    info->writeAsListing("roller_names", desc.rollerNames);
    info->writeAsListing("guide_names", desc.guideNames);

    write(info, "up_axis", desc.upAxis);

    return true;
}

SceneDevice* createSceneAGXVehicleContinuousTrackDevice(Device* device)
{
    return new SceneTrackDevice(static_cast<AGXVehicleContinuousTrackDevice*>(device));
}


struct TypeRegistration
{
    TypeRegistration() {
        StdBodyLoader::registerNodeType(
            "AGXVehicleContinuousTrackDevice", readAGXVehicleContinuousTrackDevice);
        StdBodyWriter::registerDeviceWriter<AGXVehicleContinuousTrackDevice>(
            "AGXVehicleContinuousTrackDevice", writeAGXVehicleContinuousTrackDevice);
        SceneDevice::registerSceneDeviceFactory<AGXVehicleContinuousTrackDevice>(
            createSceneAGXVehicleContinuousTrackDevice);

        if(!AGXObjectFactory::checkModuleEnalbled("AGX-Vehicle") && !AGXObjectFactory::checkModuleEnalbled("AgX-Vehicle")){
            std::cout << "Please check you have AGX-Vehicle module license."
                "AGXVehicleContinuousTrackDevice should not work." << std::endl;
        }
    }
} registration;

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDeviceDesc& desc)
    : AGXVehicleContinuousTrackDeviceDesc(desc)
{
    initialize();
}

AGXVehicleContinuousTrackDevice::AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDevice& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}

const char* AGXVehicleContinuousTrackDevice::typeName() const
{
    return "AGXVehicleContinuousTrackDevice";
}


void AGXVehicleContinuousTrackDevice::copyStateFrom(const AGXVehicleContinuousTrackDevice& other)
{
    AGXVehicleContinuousTrackDeviceDesc desc;
    AGXVehicleContinuousTrackDevice& dev = const_cast<AGXVehicleContinuousTrackDevice&>(other);
    dev.getDesc(desc);  // Need to get desc. So do const_cast above.
    setDesc(desc);
    m_trackStates = other.m_trackStates;
}


void AGXVehicleContinuousTrackDevice::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AGXVehicleContinuousTrackDevice)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AGXVehicleContinuousTrackDevice&>(other));
}


DeviceState* AGXVehicleContinuousTrackDevice::cloneState() const
{
    return new AGXVehicleContinuousTrackDevice(*this, true);
}


Referenced* AGXVehicleContinuousTrackDevice::doClone(CloneMap*) const
{
    return new AGXVehicleContinuousTrackDevice(*this);
}


void AGXVehicleContinuousTrackDevice::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AGXVehicleContinuousTrackDevice))){
        Device::forEachActualType(func);
    }
}


int AGXVehicleContinuousTrackDevice::stateSize() const
{
    return numberOfNodes * 10;
}


const double* AGXVehicleContinuousTrackDevice::readState(const double* buf)
{
    if(m_trackStates.size() < numberOfNodes){
        m_trackStates.resize(numberOfNodes);
    }
    int i = 0;
    for(int j=0; j < numberOfNodes; ++j){
        TrackState& state = m_trackStates[j];
        state.boxSize << buf[i], buf[i+1], buf[i+2];
        i += 3;
        state.position.translation() << buf[i], buf[i+1], buf[i+2];
        i += 3;
        state.position.linear() = Quaternion(buf[i], buf[i+1], buf[i+2], buf[i+3]).toRotationMatrix();
        i += 4;
    }
    return buf + i;
}

double* AGXVehicleContinuousTrackDevice::writeState(double* out_buf) const
{
    double* out_buf0 = out_buf;
    
    int i = 0;
    int j = 0;

    while(j < m_trackStates.size()){
        const TrackState& state = m_trackStates[j++];
        const auto& s = state.boxSize;
        out_buf[i++] = s.x();
        out_buf[i++] = s.y();
        out_buf[i++] = s.z();
        const auto& T = state.position;
        auto p = T.translation();
        out_buf[i++] = p.x();
        out_buf[i++] = p.y();
        out_buf[i++] = p.z();
        Quaternion q(T.linear());
        out_buf[i++] = q.w();
        out_buf[i++] = q.x();
        out_buf[i++] = q.y();
        out_buf[i++] = q.z();
    }
    out_buf += i;

    int n = numberOfNodes - j;
    if(n > 0){
        int size = n * 10;
        std::fill(out_buf, out_buf + size, 0.0);
        out_buf += size;
    }

    return out_buf;
}

void AGXVehicleContinuousTrackDevice::initialize()
{
    m_trackStates.clear();
}

Vector3 AGXVehicleContinuousTrackDevice::getUpAxis() const
{
    return upAxis;
}

int AGXVehicleContinuousTrackDevice::getNumNodes() const
{
    return numberOfNodes;
}

const vector<string> AGXVehicleContinuousTrackDevice::getSprocketNames() const
{
    return sprocketNames;
}

const vector<string> AGXVehicleContinuousTrackDevice::getIdlerNames() const
{
    return idlerNames;
}

const vector<string> AGXVehicleContinuousTrackDevice::getRollerNames() const
{
    return rollerNames;
}

void AGXVehicleContinuousTrackDevice::setDesc(const AGXVehicleContinuousTrackDeviceDesc& desc){
    static_cast<AGXVehicleContinuousTrackDeviceDesc&>(*this) = desc;
}

void AGXVehicleContinuousTrackDevice::getDesc(AGXVehicleContinuousTrackDeviceDesc& desc) const
{
    desc = static_cast<const AGXVehicleContinuousTrackDeviceDesc&>(*this);
}

void AGXVehicleContinuousTrackDevice::reserveTrackStateSize(const unsigned int& num) {
    m_trackStates.reserve(num);
}

void AGXVehicleContinuousTrackDevice::addTrackState(const Vector3& boxSize, const Isometry3& pos) {
    TrackState s;
    s.boxSize = boxSize;
    s.position = pos;
    m_trackStates.push_back(s);
}

TrackStates& AGXVehicleContinuousTrackDevice::getTrackStates() {
    return m_trackStates;
}

SgShape* AGXVehicleContinuousTrackDevice::getNodeShape(){
    return nodeShape;
}

}
