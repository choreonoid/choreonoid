#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_TRACK_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_TRACK_H

#include <cnoid/Device>
#include <cnoid/SceneDrawables>
#include <string>
#include <vector>

using namespace std;
namespace cnoid {

struct AGXVehicleContinuousTrackDeviceDesc
{
    AGXVehicleContinuousTrackDeviceDesc() {
        upAxis = Vector3d(0.0, 0.0, 1.0);
        numberOfNodes = 50;
        nodeThickness = 0.075;
        nodeWidth = 0.6;
        nodeDistanceTension = 5.0E-3;
        nodeThickerThickness = 0.09;
        useThickerNodeEvery = 0;
        hingeCompliance = 1.0E-10;
        hingeSpookDamping = 0.0333;
        minStabilizingHingeNormalForce = 100;
        stabilizingHingeFrictionParameter = 1.5;
        nodesToWheelsMergeThreshold = -0.1;
        nodesToWheelsSplitThreshold = -0.05;
        enableMerge = false;
        numNodesPerMergeSegment = 3;
        contactReduction = 1;
        enableLockToReachMergeCondition = false;
        lockToReachMergeConditionCompliance = 1.0E-11;
        lockToReachMergeConditionSpookDamping = 3/ 60;
        maxAngleMergeCondition = 1.0E-5;
        sprocketNames.clear();
        idlerNames.clear();
        rollerNames.clear();
        guideNames.clear();
    }
    Vector3 upAxis;
    int numberOfNodes;           // Total number of nodes in the track.
    double nodeThickness;        // Thickness of each node in the track.
    double nodeWidth;            // Width of each node in the track.
    double nodeDistanceTension;  // The calculated node length is close to ideal, meaning close to zero tension
                                 // in the tracks if they were simulated without gravity. This distance is an offset
                                 // how much closer each node will be to each other, resulting in a given initial tension.

    double nodeThickerThickness;
    int useThickerNodeEvery;
    double hingeCompliance;
    double hingeSpookDamping;
    double minStabilizingHingeNormalForce;
    double stabilizingHingeFrictionParameter;
    double nodesToWheelsMergeThreshold;
    double nodesToWheelsSplitThreshold;
    bool enableMerge;
    int numNodesPerMergeSegment;
    int contactReduction;
    bool enableLockToReachMergeCondition;
    double lockToReachMergeConditionCompliance;
    double lockToReachMergeConditionSpookDamping;
    double maxAngleMergeCondition;
    vector<string> sprocketNames;
    vector<string> idlerNames;
    vector<string> rollerNames;
    vector<string> guideNames;
    string materialName;
    SgShapePtr nodeShape;
};

struct TrackState{
        Vector3 boxSize;
        Position position;
};
typedef std::vector<TrackState> TrackStates;

class AGXVehicleContinuousTrackDevice : private AGXVehicleContinuousTrackDeviceDesc, public Device
{
public:
    AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDeviceDesc& desc);
    AGXVehicleContinuousTrackDevice(const AGXVehicleContinuousTrackDevice& org, bool copyStateOnly = false);
    virtual const char* typeName() override;
    void copyStateFrom(const AGXVehicleContinuousTrackDevice& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual Device* clone() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    void initialize();
    Vector3 getUpAxis() const;
    int getNumNodes() const;
    const vector<string> getSprocketNames() const;
    const vector<string> getIdlerNames() const;
    const vector<string> getRollerNames() const;
    void setDesc(const AGXVehicleContinuousTrackDeviceDesc& desc);
    void getDesc(AGXVehicleContinuousTrackDeviceDesc& desc);
    void reserveTrackStateSize(const unsigned int& num );
    void addTrackState(const Vector3& boxSize, const Position& pos);
    TrackStates& getTrackStates();
    SgShape* getNodeShape();

private:
    TrackStates m_trackStates;
};

typedef ref_ptr<AGXVehicleContinuousTrackDevice> AGXVehicleContinuousTrackDevicePtr;

}


#endif
