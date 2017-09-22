#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_TRACK_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_VEHICLE_TRACK_H

#include <cnoid/Device>
#include <string>
#include <vector>

using namespace std;
namespace cnoid {

struct AGXVehicleContinuousTrackDeviceDesc{
    AGXVehicleContinuousTrackDeviceDesc() {
        upAxis = Vector3d(0.0, 0.0, 1.0);
        numberOfNodes = 50;
        nodeThickness = 0.075;
        nodeWidth = 0.6;
        nodeDistanceTension = 5.0E-3;
        hingeCompliance = 4.0E-10;
        stabilizingHingeFrictionParameter = 1.5;
        enableMerge = false;
        numNodesPerMergeSegment = 4;
        lockToReachMergeConditionCompliance = 1.0E-8;
        contactReductionLevel = 3;
        sprocketNames.clear();
        idlerNames.clear();
        rollerNames.clear();
    }
    Vector3 upAxis;
    int numberOfNodes;           // Total number of nodes in the track.
    double nodeThickness;        // Thickness of each node in the track.
    double nodeWidth;            // Width of each node in the track.
    double nodeDistanceTension;  // The calculated node length is close to ideal, meaning close to zero tension
                                              // in the tracks if they were simulated without gravity. This distance is an offset
                                              // how much closer each node will be to each other, resulting in a given initial tension.
    double hingeCompliance;
    double stabilizingHingeFrictionParameter;
    bool enableMerge;
    int numNodesPerMergeSegment;
    double lockToReachMergeConditionCompliance;
    int contactReductionLevel;
    vector<string> sprocketNames;
    vector<string> idlerNames;
    vector<string> rollerNames;
};

struct TrackState{
        Vector3 boxSize;
        Affine3 transform;
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

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
    Vector3 getUpAxis() const;
    int getNumNodes() const;
    int numSprocketNames() const;
    const string* getSprocketNames() const;
    int numIdlerNames() const;
    const string* getIdlerNames() const;
    int numRollerNames() const;
    const string* getRollerNames() const;
    void setDesc(const AGXVehicleContinuousTrackDeviceDesc& desc);
    void getDesc(AGXVehicleContinuousTrackDeviceDesc& desc);
    void reserveTrackStateSize(const unsigned int& num );
    void addTrackState(const Vector3& boxSize, const Position& transform);
    TrackStates& getTrackStates();

private:
    bool on_;
    TrackStates _trackStates;
};

typedef ref_ptr<AGXVehicleContinuousTrackDevice> AGXVehicleContinuousTrackDevicePtr;

}


#endif