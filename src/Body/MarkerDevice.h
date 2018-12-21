/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_MARKER_DEVICE_H
#define CNOID_BODY_MARKER_DEVICE_H

#include <cnoid/Device>
#include "exportdecl.h"

namespace cnoid {

class YAMLBodyLoader;
class Mapping;

class CNOID_EXPORT MarkerDevice : public Device
{
public:
    MarkerDevice();
    MarkerDevice(const MarkerDevice& org, bool copyStateOnly = false);
    
    virtual const char* typeName();    
    void copyMarkerDeviceStateFrom(const MarkerDevice& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func);
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

    bool readDescription(YAMLBodyLoader& loader, Mapping& node);

    virtual bool on() const;
    virtual void on(bool on);

    enum MarkerType {
        CROSS_MARKER,
        SPHERE_MARKER,
        AXES_MARKER,
        N_MARKER_TYPES
    };

    int markerType() const { return markerType_; }
    void setMarkerType(int type){ markerType_ = type; }

    double markerSize() const { return markerSize_; }
    void setMarkerSize(double size) { markerSize_ = size; }
        
    const Vector3f& color() const { return color_; }
    void setColor(const Vector3f& c) { color_ = c; }
    float emission() const { return emission_; }
    void setEmission(float r) { emission_ = r; }
    float transparency() const { return transparency_; }
    void setTransparency(float t) { transparency_ = t; }

    const Position& offsetPosition() const { return offsetPosition_; }
    void setOffsetPosition(const Position& T) { offsetPosition_ = T; }
    void setOffsetTranslation(const Vector3& p) { offsetPosition_.translation() = p; }

private:
    bool on_;
    int markerType_;
    float markerSize_;
    Position offsetPosition_;
    Vector3f color_;
    float emission_;
    float transparency_;
};

typedef ref_ptr<MarkerDevice> MarkerDevicePtr;

}

#endif
