#ifndef CNOID_PHYSX_PLUGIN_PX_CONTINUOUS_TRACK_H
#define CNOID_PHYSX_PLUGIN_PX_CONTINUOUS_TRACK_H

#include <cnoid/Device>
#include <cnoid/EigenTypes>
#include <string>
#include <vector>
#include <memory>

namespace cnoid {

class SgMaterial;

class PxContinuousTrack : public Device
{
public:
    PxContinuousTrack();
    PxContinuousTrack(const PxContinuousTrack& org, bool copyStateOnly = false);
    virtual const char* typeName() const override;
    void copyStateFrom(const PxContinuousTrack& other);
    virtual void copyStateFrom(const DeviceState& other) override;
    virtual DeviceState* cloneState() const override;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func) override;
    virtual void clearState() override;
    virtual int stateSize() const override;
    virtual const double* readState(const double* buf, int size) override;
    virtual double* writeState(double* out_buf) const override;

    struct Spec {
        std::string sprocketName;
        double sprocketRadius;
        std::string idlerName;
        double idlerRadius;
        int numShoes; // user-specified value
        double grouserHeight;
        double grouserThickness;
        double thickness;
        double width;
        std::string contactMaterialName;
        ref_ptr<SgMaterial> appearanceMaterial;
        double mass;
        double driveDamping;
        double driveMaxForce;

        // Computed geometry data (set by clearState)

        // The number of visible shoes varies slightly per frame because wheel
        // rotation shifts which grousers pass the half-arc visibility test.
        // This fixed upper-bound is used for state I/O buffer allocation so
        // that stateSize() remains constant across simulation steps.
        int maxNumVisibleShoes;
        double wheelDistance;
        double grouserSpacing;
        double avgWheelRadius;
        double effectiveSprocketRadius;
        double effectiveIdlerRadius;
        int numGrousersPerSegment;
        int numGrousersOnSprocket;
        int numGrousersOnIdler;
        Vector3 upperSegmentCenter;
        Vector3 lowerSegmentCenter;
        Vector3 slideDir;
        double segmentLength;
        bool geometryInitialized;
        int sprocketFullGrousers;
        int idlerFullGrousers;
        double angleStepSprocket;
        double angleStepIdler;
        int sprocketBeltSegments;
        int idlerBeltSegments;
        double angleStepSprocketBelt;
        double angleStepIdlerBelt;

        Spec();
    };

    const Spec& spec() const { return *spec_; }

    // Spec accessors
    const std::string& sprocketName() const { return spec_->sprocketName; }
    void setSprocketName(const std::string& name) { spec_->sprocketName = name; }
    double sprocketRadius() const { return spec_->sprocketRadius; }
    void setSprocketRadius(double r) { spec_->sprocketRadius = r; }

    const std::string& idlerName() const { return spec_->idlerName; }
    void setIdlerName(const std::string& name) { spec_->idlerName = name; }
    double idlerRadius() const { return spec_->idlerRadius; }
    void setIdlerRadius(double r) { spec_->idlerRadius = r; }

    int numShoes() const { return spec_->numShoes; }
    void setNumShoes(int n) { spec_->numShoes = n; }
    double grouserHeight() const { return spec_->grouserHeight; }
    void setGrouserHeight(double h) { spec_->grouserHeight = h; }
    double grouserThickness() const { return spec_->grouserThickness; }
    void setGrouserThickness(double t) { spec_->grouserThickness = t; }
    double effectiveGrouserThickness() const {
        return (spec_->grouserThickness > 0.0) ? spec_->grouserThickness : spec_->thickness;
    }

    double thickness() const { return spec_->thickness; }
    void setThickness(double t) { spec_->thickness = t; }
    double width() const { return spec_->width; }
    void setWidth(double w) { spec_->width = w; }

    const std::string& contactMaterialName() const { return spec_->contactMaterialName; }
    void setContactMaterialName(const std::string& name) { spec_->contactMaterialName = name; }

    SgMaterial* appearanceMaterial() const { return spec_->appearanceMaterial; }
    void setAppearanceMaterial(SgMaterial* material) { spec_->appearanceMaterial = material; }

    double mass() const { return spec_->mass; }
    void setMass(double m) { spec_->mass = m; }

    double driveDamping() const { return spec_->driveDamping; }
    void setDriveDamping(double d) { spec_->driveDamping = d; }
    double driveMaxForce() const { return spec_->driveMaxForce; }
    void setDriveMaxForce(double f) { spec_->driveMaxForce = f; }

    bool isGeometryInitialized() const { return spec_->geometryInitialized; }

    // State accessors
    int numVisibleShoes() const { return static_cast<int>(shoePositions_.size()); }

    const std::vector<SE3>& shoePositions() const { return shoePositions_; }
    std::vector<SE3>& shoePositions() { return shoePositions_; }
    void addShoePosition(const SE3& pos);
    void clearShoePositions();

protected:
    virtual Referenced* doClone(CloneMap* cloneMap) const override;

private:
    // State
    std::vector<SE3> shoePositions_;

    // Spec
    std::unique_ptr<Spec> spec_;
};

typedef ref_ptr<PxContinuousTrack> PxContinuousTrackPtr;

}

#endif
