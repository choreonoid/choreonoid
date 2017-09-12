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
        sprocketNames.clear();
        idlerNames.clear();
        rollerNames.clear();
        //sprocketRadius = 0.45;
        //sprocketHeight = 0.35;
        //idlerRadius = 0.40;   
        //idlerHeight = 0.35;   
        //rollerRadius = 0.35;  
        //rollerHeight = 0.35;  
        //rollerOffset = 1.0 * rollerRadius;
        //numRollers = 6;
    }
    Vector3 upAxis;
    int numberOfNodes;        // Total number of nodes in the track.
    double nodeThickness;        // Thickness of each node in the track.
    double nodeWidth;            // Width of each node in the track.
    double nodeDistanceTension;  // The calculated node length is close to ideal, meaning close to zero tension
                                              // in the tracks if they were simulated without gravity. This distance is an offset
                                              // how much closer each node will be to each other, resulting in a given initial tension.
                                              //agx::Real sprocketRadius;       // Radius of the sprocket wheel.
                                              //agx::Real sprocketHeight;       // Height of the sprocket wheel.
    vector<string> sprocketNames;
    vector<string> idlerNames;
    vector<string> rollerNames;
    //agx::Real idlerRadius;           // Radius of the idler wheel.
    //agx::Real idlerHeight;           // Height of the idler wheel.
    //agx::Real rollerRadius;          // Radius of the ground/roller wheel(s).
    //agx::Real rollerHeight;          // Height of the ground/roller wheel(s).
    //agx::Real rollerOffset;          // Offset between the rollers.
    //agx::UInt numRollers;            // Number of rollers/ground wheels.
};

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
    int getNumberOfNodes() const;
    double getNodeThickness() const;
    double getNodeWidth() const;
    double getNodeDistanceTension() const;
    int numSprocketNames() const;
    const string* getSprocketNames() const;
    int numIdlerNames() const;
    const string* getIdlerNames() const;
    int numRollerNames() const;
    const string* getRollerNames() const;


private:
    bool on_;
};

typedef ref_ptr<AGXVehicleContinuousTrackDevice> AGXVehicleContinuousTrackDevicePtr;

}


#endif