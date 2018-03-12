/**
   @author Japan Atomic Energy Agency
*/

#pragma once
#include <map>
#include "exportdecl.h"

namespace Multicopter{

class CNOID_EXPORT RotorDevice : public cnoid::Device
{
public:

    bool isActive() const { return _on; }
    void setActive(bool flg) { _on = flg; }

    Eigen::Vector3d position() const;
    void setPosition(const Eigen::Vector3d& pos);
    Eigen::Vector3d direction() const;
    void setDirection(const Eigen::Vector3d& dir);

    double value() const;
    void setValue(const double val);
    void valueRange(double& min, double& max) const;
    void setValueRange(const double min, const double max);
    double torque()const;
    void setTorque(const double torque);
    void torqueRange(double& min, double& max) const;
    void setTorqueRange(const double min, const double max);

    double wallEffectDistance() const;
    void setWallEffectDistance(double wallEffectDistance);

    double wallEffectNormMiddleValue() const;
    void setWallEffectNormMiddleValue(double wallEffectNormMiddleValue);

    double wallEffectMaxRate() const;
    void setWallEffectMaxRate(double wallEffectMaxRate);

    double groundEffectDistance() const;
    void setGroundEffectDistance(double groundEffectDistance);

    double groundEffectNormMiddleValue() const;
    void setGroundEffectNormMiddleValue(double groundEffectNormMiddleValue);

    double groundEffectMaxRate() const;
    void setGroundEffectMaxRate(double groundEffectMaxRate);

    double applyValueRange(const double val);
    double applyTorqueRange(const double val);

    void showParameter();

    RotorDevice();
    RotorDevice(const RotorDevice& org,  bool copyStateOnly = false);

    virtual const char* typeName();
    void copyStateFrom(const RotorDevice& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func);
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;


private:
    bool _on;
    Eigen::Vector3d _pos;
    Eigen::Vector3d _dir;
    double _val;
    double _minVal;
    double _maxVal;
    double _torque;
    double _torqueMinVal;
    double _torqueMaxVal;
    double _wallEffectDistance;
    double _wallEffectNormMiddleValue;
    double _wallEffectMaxRate;
    double _groundEffectDistance;
    double _groundEffectNormMiddleValue;
    double _groundEffectMaxRate;
};

typedef cnoid::ref_ptr<RotorDevice> RotorDevicePtr;

}
