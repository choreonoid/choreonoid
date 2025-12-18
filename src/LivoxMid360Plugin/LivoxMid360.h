#ifndef CNOID_LIVOX_MID360_PLUGIN_LIVOX_MID360_H
#define CNOID_LIVOX_MID360_PLUGIN_LIVOX_MID360_H

#include <cnoid/RangeSensor>
#include <cnoid/EigenTypes>
#include <filesystem>
#include <vector>

namespace cnoid {

class LivoxMid360 : public RangeSensor
{
public:
    LivoxMid360();
    LivoxMid360(const LivoxMid360& org, bool copyStateOnly = false);

    virtual const char* typeName() const override;
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;

    double angularPrecision() const { return angularPrecision_; }
    void setAngularPrecision(double p) { angularPrecision_ = p; }
    int numSamples() const { return numSamples_; }
    void setNumSamples(int n);
    
    const std::vector<Vector2f>& sphericalAngleSeq() const { return *sphericalAngleSeq_; }
    
    bool readSpecifications(const Mapping* info, const std::filesystem::path& baseDirPath);
    bool loadSphericalAngleSeqFile(const std::string& filename);
    bool writeSpecifications(Mapping* info) const;


protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    int numSamples_;
    double angularPrecision_;
    std::string sphericalAngleSeqFilename;
    std::shared_ptr<std::vector<Vector2f>> sphericalAngleSeq_;

    void copyLivoxMid360StateFrom(const LivoxMid360& other, bool doCopyRangeSensorState, bool doCopyRangeData);
};

typedef ref_ptr<LivoxMid360> LivoxMid360Ptr;

}

#endif
