#ifndef CNOID_BODY_VISION_SENSOR_H
#define CNOID_BODY_VISION_SENSOR_H

#include "Device.h"
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class Mapping;

class CNOID_EXPORT VisionSensor : public Device
{
public:
    VisionSensor();
    VisionSensor(const VisionSensor& org, bool copyStateOnly = false);
    void copyVisionSensorStateFrom(const VisionSensor& other); 
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual bool on() const override;
    virtual void on(bool on) override;

    const Matrix3& opticalFrameRotation() const { return spec->R_optical; }
    template<typename Derived>
    void setOpticalFrameRotation(const Eigen::MatrixBase<Derived>& R) { spec->R_optical = R; }
    
    double frameRate() const { return frameRate_; }
    void setFrameRate(double r) { frameRate_ = r; }
    
    /**
       Time [s] consumed in shooting the current image
    */
    double delay() const { return delay_; }
    void setDelay(double time) { delay_ = time; }

    static int visionSensorStateSize();
    virtual const double* readState(const double* buf) override;
    virtual double* writeState(double* out_buf) const override;

    bool readSpecifications(const Mapping* info);
    bool writeSpecifications(Mapping* info) const;

private:
    bool on_;
    double frameRate_;
    double delay_;

    struct Spec {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Matrix3 R_optical;
    };
    std::unique_ptr<Spec> spec;
};

typedef ref_ptr<VisionSensor> VisionSensorPtr;

}

#endif
