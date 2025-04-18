#ifndef CNOID_BODY_POINT_LIGHT_H
#define CNOID_BODY_POINT_LIGHT_H

#include "Light.h"
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT PointLight : public Light
{
public:
    PointLight();
    PointLight(const PointLight& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    void copyStateFrom(const PointLight& other, bool doCopyLightState = true);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;

    static int pointLightStateSize();
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    float constantAttenuation() const { return constantAttenuation_; }
    void setConstantAttenuation(float a) { constantAttenuation_ = a; }

    float linearAttenuation() const { return linearAttenuation_; }
    void setLinearAttenuation(float a) { linearAttenuation_ = a; }

    float quadraticAttenuation() const { return quadraticAttenuation_; }
    void setQuadraticAttenuation(float a) { quadraticAttenuation_ = a; }

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;
        
private:
    float constantAttenuation_;
    float linearAttenuation_;
    float quadraticAttenuation_;
};

typedef ref_ptr<PointLight> PointLightPtr;

}

#endif
