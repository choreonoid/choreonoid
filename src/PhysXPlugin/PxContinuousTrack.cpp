#include "PxContinuousTrack.h"
#include <cnoid/StdBodyLoader>
#include <cnoid/StdBodyWriter>
#include <cnoid/StdSceneReader>
#include <cnoid/SceneDevice>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/Link>
#include <cnoid/Body>
#include <cnoid/MessageOut>
#include <cnoid/Format>
#include <cmath>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ScenePxContinuousTrack : public SceneDevice
{
    PxContinuousTrack* trackDevice;
    SgSwitchableGroupPtr trackSwitch;
    SgUpdate sgUpdate;

    // Grouser shapes
    std::vector<SgPosTransformPtr> sgShoes;
    bool shoesCreated;

    // Plate shapes (upper, lower)
    SgPosTransformPtr sgUpperPlate;
    SgPosTransformPtr sgLowerPlate;
    bool platesCreated;

    // Belt shapes (sprocket front half, idler back half)
    SgGroupPtr sgSprocketBelt;
    SgGroupPtr sgIdlerBelt;
    bool beltsCreated;

public:
    ScenePxContinuousTrack(PxContinuousTrack* device)
        : SceneDevice(device),
          trackDevice(device),
          shoesCreated(false),
          platesCreated(false),
          beltsCreated(false)
    {
        trackSwitch = new SgSwitchableGroup;
        trackSwitch->setTurnedOn(false);
        addChild(trackSwitch);
        setFunctionOnStateChanged([this](){ update(); });
    }

    void update()
    {
        auto& spec = trackDevice->spec();
        if(!spec.geometryInitialized){
            trackSwitch->setTurnedOn(false, sgUpdate);
            return;
        }

        if(!platesCreated){
            createPlateShapes();
            platesCreated = true;
        }

        if(!beltsCreated){
            createBeltShapes();
            beltsCreated = true;
        }

        auto& shoes = trackDevice->shoePositions();
        if(shoes.empty()){
            trackSwitch->setTurnedOn(false, sgUpdate);
            return;
        }

        if(!shoesCreated || static_cast<int>(shoes.size()) != static_cast<int>(sgShoes.size())){
            createShoeShapes();
            shoesCreated = true;
        }
        trackSwitch->setTurnedOn(true, sgUpdate);
        updatePositions();
    }

    SgMaterial* createTrackMaterial()
    {
        if(auto specMat = trackDevice->appearanceMaterial()){
            return new SgMaterial(*specMat);
        }
        auto material = new SgMaterial;
        material->setDiffuseColor(Vector3f(0.15f, 0.15f, 0.15f));
        return material;
    }

    void createPlateShapes()
    {
        auto& spec = trackDevice->spec();
        MeshGenerator meshGenerator;
        Vector3 plateSize(spec.segmentLength, spec.width, spec.thickness);

        auto material = createTrackMaterial();

        // Upper plate
        {
            auto sgPlate = new SgPosTransform;
            auto shape = new SgShape;
            shape->setMesh(meshGenerator.generateBox(plateSize));
            shape->setMaterial(material);
            sgPlate->addChild(shape);
            trackSwitch->addChild(sgPlate);
            sgUpperPlate = sgPlate;
        }
        // Lower plate
        {
            auto sgPlate = new SgPosTransform;
            auto shape = new SgShape;
            shape->setMesh(meshGenerator.generateBox(plateSize));
            shape->setMaterial(material);
            sgPlate->addChild(shape);
            trackSwitch->addChild(sgPlate);
            sgLowerPlate = sgPlate;
        }
    }

    void createShoeShapes()
    {
        auto& spec = trackDevice->spec();
        auto& shoes = trackDevice->shoePositions();
        MeshGenerator meshGenerator;

        // Remove old shoe shapes (keep plates at indices 0,1)
        for(auto& sg : sgShoes){
            trackSwitch->removeChild(sg);
        }
        sgShoes.clear();
        sgShoes.reserve(shoes.size());

        Vector3 shoeSize(trackDevice->effectiveGrouserThickness(), spec.width, spec.grouserHeight);

        auto material = createTrackMaterial();

        for(size_t i = 0; i < shoes.size(); ++i){
            auto sgShoe = new SgPosTransform;
            auto shape = new SgShape;
            shape->setMesh(meshGenerator.generateBox(shoeSize));
            shape->setMaterial(material);
            sgShoe->addChild(shape);
            trackSwitch->addChild(sgShoe);
            sgShoes.push_back(sgShoe);
        }
    }

    void createBeltShapes()
    {
        auto& spec = trackDevice->spec();
        MeshGenerator meshGenerator;
        meshGenerator.setDivisionNumber(spec.sprocketBeltSegments * 2);
        auto material = createTrackMaterial();

        // generateRing uses phi=0 -> +x, phi rotates in x-z plane.
        // In the track coordinate system, theta=PI/2 is "up" (z+),
        // theta=0 is "front" (slideDir+).
        // The ring's phi=0 corresponds to +x direction.
        // We need to map ring's +x to slideDir, so we build a rotation
        // that maps +x -> slideDir (in the x-z plane).

        // Build rotation matrix: ring's x-z plane -> track's slideDir-z plane
        // slideDir is in x-z plane (y=0), so rotation around Y axis
        double slideDirAngle = atan2(spec.slideDir.z(), spec.slideDir.x());

        // Sprocket belt (front half-circle)
        // In the ring coordinate (before rotation), the front half is
        // where cos(phi) >= 0, i.e. phi in [-PI/2, PI/2].
        // But the sprocket's visible arc is the outer half where
        // theta goes from PI/2 (top) through 0 (front) to -PI/2 (bottom).
        // In ring coordinates, this corresponds to phi from PI/2 to -PI/2,
        // which is equivalent to generating from -PI/2 to PI/2
        // (the half where cos(phi) >= 0).
        sgSprocketBelt = new SgGroup;
        {
            double r_belt = spec.sprocketRadius + spec.thickness / 2.0;
            auto mesh = meshGenerator.generateRing(
                r_belt, spec.width, spec.thickness, -M_PI / 2.0, M_PI / 2.0);

            Vector3 wheelCenter = spec.upperSegmentCenter
                + spec.slideDir * (spec.wheelDistance / 2.0);
            wheelCenter.z() -= spec.avgWheelRadius;

            auto sgBelt = new SgPosTransform;
            Isometry3 T;
            T.setIdentity();
            T.linear() = Eigen::AngleAxisd(slideDirAngle, Vector3::UnitY()).toRotationMatrix();
            T.translation() = wheelCenter;
            sgBelt->setTransform(T);

            auto shape = new SgShape;
            shape->setMesh(mesh);
            shape->setMaterial(material);
            sgBelt->addChild(shape);
            sgSprocketBelt->addChild(sgBelt);
            trackSwitch->addChild(sgSprocketBelt);
        }

        // Idler belt (back half-circle)
        // The idler's visible arc is where cos(theta) <= 0,
        // i.e. theta from PI/2 (top) through PI (back) to 3PI/2=-PI/2 (bottom).
        // In ring coordinates this is phi from PI/2 to 3PI/2.
        sgIdlerBelt = new SgGroup;
        {
            meshGenerator.setDivisionNumber(spec.idlerBeltSegments * 2);
            double r_belt = spec.idlerRadius + spec.thickness / 2.0;
            auto mesh = meshGenerator.generateRing(
                r_belt, spec.width, spec.thickness, M_PI / 2.0, 3.0 * M_PI / 2.0);

            Vector3 wheelCenter = spec.upperSegmentCenter
                - spec.slideDir * (spec.wheelDistance / 2.0);
            wheelCenter.z() -= spec.avgWheelRadius;

            auto sgBelt = new SgPosTransform;
            Isometry3 T;
            T.setIdentity();
            T.linear() = Eigen::AngleAxisd(slideDirAngle, Vector3::UnitY()).toRotationMatrix();
            T.translation() = wheelCenter;
            sgBelt->setTransform(T);

            auto shape = new SgShape;
            shape->setMesh(mesh);
            shape->setMaterial(material);
            sgBelt->addChild(shape);
            sgIdlerBelt->addChild(sgBelt);
            trackSwitch->addChild(sgIdlerBelt);
        }
    }

    void updatePositions()
    {
        auto& spec = trackDevice->spec();
        auto& shoes = trackDevice->shoePositions();

        // Update plate positions (fixed in link local frame)
        if(sgUpperPlate){
            Isometry3 T;
            T.setIdentity();
            T.translation() = spec.upperSegmentCenter;
            T.translation().z() += spec.thickness / 2.0;
            sgUpperPlate->setTransform(T);
        }
        if(sgLowerPlate){
            Isometry3 T;
            T.setIdentity();
            T.translation() = spec.lowerSegmentCenter;
            T.translation().z() -= spec.thickness / 2.0;
            sgLowerPlate->setTransform(T);
        }

        // Update shoe positions (already in track link local frame)
        for(size_t i = 0; i < shoes.size() && i < sgShoes.size(); ++i){
            Isometry3 T;
            T.linear() = shoes[i].rotation().toRotationMatrix();
            T.translation() = shoes[i].translation();
            sgShoes[i]->setTransform(T);
        }
        notifyUpdate(sgUpdate);
    }
};


bool readPxContinuousTrack(StdBodyLoader* loader, const Mapping* info)
{
    PxContinuousTrackPtr device = new PxContinuousTrack;

    string name;
    if(info->read("name", name)){
        device->setName(name);
    }

    string sprocketName;
    if(info->read("sprocket", sprocketName)){
        device->setSprocketName(sprocketName);
    }
    double sprocketRadius = 0.0;
    if(info->read("sprocket_radius", sprocketRadius)){
        device->setSprocketRadius(sprocketRadius);
    }

    string idlerName;
    if(info->read("idler", idlerName)){
        device->setIdlerName(idlerName);
    }
    double idlerRadius = 0.0;
    if(info->read("idler_radius", idlerRadius)){
        device->setIdlerRadius(idlerRadius);
    }

    int numShoes = 20;
    info->read("num_shoes", numShoes);
    device->setNumShoes(numShoes);

    double grouserHeight = 0.01;
    info->read("grouser_height", grouserHeight);
    device->setGrouserHeight(grouserHeight);

    double grouserThickness = 0.0;
    info->read("grouser_thickness", grouserThickness);
    device->setGrouserThickness(grouserThickness);

    double thickness = 0.005;
    info->read("thickness", thickness);
    device->setThickness(thickness);

    double width = 0.1;
    info->read("width", width);
    device->setWidth(width);

    string contactMaterialName;
    if(info->read("contact_material", contactMaterialName)){
        device->setContactMaterialName(contactMaterialName);
    }

    auto appearanceInfo = info->findMapping("appearance_material");
    if(appearanceInfo->isValid()){
        if(auto material = loader->sceneReader()->readMaterial(appearanceInfo)){
            device->setAppearanceMaterial(material);
        }
    }

    double mass = 0.0;
    info->read("mass", mass);
    device->setMass(mass);

    double driveDamping = 1.0e4;
    info->read("drive_damping", driveDamping);
    device->setDriveDamping(driveDamping);

    double driveMaxForce = 0.0;
    info->read("drive_max_force", driveMaxForce);
    device->setDriveMaxForce(driveMaxForce);

    return loader->readDevice(device, info);
}


bool writePxContinuousTrack(
    StdBodyWriter* writer, Mapping* info, const PxContinuousTrack* device)
{
    if(!device->sprocketName().empty()){
        info->write("sprocket", device->sprocketName());
    }
    info->write("sprocket_radius", device->sprocketRadius());
    if(!device->idlerName().empty()){
        info->write("idler", device->idlerName());
    }
    info->write("idler_radius", device->idlerRadius());
    info->write("num_shoes", device->numShoes());
    info->write("grouser_height", device->grouserHeight());
    if(device->grouserThickness() > 0.0){
        info->write("grouser_thickness", device->grouserThickness());
    }
    info->write("thickness", device->thickness());
    info->write("width", device->width());
    if(!device->contactMaterialName().empty()){
        info->write("contact_material", device->contactMaterialName());
    }
    if(auto material = device->appearanceMaterial()){
        if(auto matInfo = writer->sceneWriter()->writeMaterial(material)){
            info->insert("appearance_material", matInfo);
        }
    }
    if(device->mass() > 0.0){
        info->write("mass", device->mass());
    }
    if(device->driveDamping() != 1.0e4){
        info->write("drive_damping", device->driveDamping());
    }
    if(device->driveMaxForce() > 0.0){
        info->write("drive_max_force", device->driveMaxForce());
    }
    return true;
}


SceneDevice* createScenePxContinuousTrack(Device* device)
{
    return new ScenePxContinuousTrack(static_cast<PxContinuousTrack*>(device));
}


struct TypeRegistration
{
    TypeRegistration() {
        StdBodyLoader::registerNodeType(
            "PxContinuousTrack", readPxContinuousTrack);
        StdBodyWriter::registerDeviceWriter<PxContinuousTrack>(
            "PxContinuousTrack", writePxContinuousTrack);
        SceneDevice::registerSceneDeviceFactory<PxContinuousTrack>(
            createScenePxContinuousTrack);
    }
} registration;


PxContinuousTrack::Spec::Spec()
    : sprocketRadius(0.0),
      idlerRadius(0.0),
      numShoes(20),
      grouserHeight(0.01),
      grouserThickness(0.0),
      thickness(0.005),
      width(0.1),
      mass(0.0),
      driveDamping(1.0e4),
      driveMaxForce(0.0),
      maxNumVisibleShoes(0),
      wheelDistance(0.0),
      grouserSpacing(0.0),
      avgWheelRadius(0.0),
      numGrousersPerSegment(0),
      numGrousersOnSprocket(0),
      numGrousersOnIdler(0),
      upperSegmentCenter(Vector3::Zero()),
      lowerSegmentCenter(Vector3::Zero()),
      slideDir(Vector3::UnitX()),
      segmentLength(0.0),
      geometryInitialized(false),
      sprocketFullGrousers(0),
      idlerFullGrousers(0),
      angleStepSprocket(0.0),
      angleStepIdler(0.0),
      sprocketBeltSegments(0),
      idlerBeltSegments(0),
      angleStepSprocketBelt(0.0),
      angleStepIdlerBelt(0.0)
{
}


PxContinuousTrack::PxContinuousTrack()
    : spec_(new Spec)
{
}


PxContinuousTrack::PxContinuousTrack(const PxContinuousTrack& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec_.reset(new Spec);
        if(org.spec_){
            *spec_ = *org.spec_;
        }
    }
}


const char* PxContinuousTrack::typeName() const
{
    return "PxContinuousTrack";
}


void PxContinuousTrack::copyStateFrom(const PxContinuousTrack& other)
{
    shoePositions_ = other.shoePositions_;
}


void PxContinuousTrack::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(PxContinuousTrack)){
        throw std::invalid_argument("Type mismatch in PxContinuousTrack::copyStateFrom");
    }
    copyStateFrom(static_cast<const PxContinuousTrack&>(other));
}


DeviceState* PxContinuousTrack::cloneState() const
{
    return new PxContinuousTrack(*this, true);
}


Referenced* PxContinuousTrack::doClone(CloneMap*) const
{
    return new PxContinuousTrack(*this);
}


void PxContinuousTrack::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(PxContinuousTrack))){
        Device::forEachActualType(func);
    }
}


void PxContinuousTrack::clearState()
{
    shoePositions_.clear();

    if(!spec_){
        return;
    }

    spec_->geometryInitialized = false;

    // Need link() to be available and body to be set
    Link* trackLink = link();
    if(!trackLink || !trackLink->body()){
        return;
    }

    Body* body = trackLink->body();
    Link* sprocketLink = body->link(spec_->sprocketName);
    Link* idlerLink = body->link(spec_->idlerName);
    if(!sprocketLink || !idlerLink){
        return;
    }

    double sprocketRadius = spec_->sprocketRadius;
    double idlerRadius = spec_->idlerRadius;
    double avgWheelRadius = (sprocketRadius + idlerRadius) / 2.0;
    double effectiveSprocketRadius = sprocketRadius + spec_->thickness;
    double effectiveIdlerRadius = idlerRadius + spec_->thickness;

    // Positions in track link local frame
    Vector3 sprocketPos = sprocketLink->Tb().translation();
    Vector3 idlerPos = idlerLink->Tb().translation();

    Vector3 diff = sprocketPos - idlerPos;
    double wheelDistance = diff.norm();

    if(wheelDistance < 1.0e-6){
        return;
    }

    double trackCircumference =
        2.0 * wheelDistance +
        M_PI * effectiveSprocketRadius +
        M_PI * effectiveIdlerRadius;
    double grouserSpacing = trackCircumference / spec_->numShoes;

    // Phase alignment check
    // Maximum allowable phase error as a fraction of grouser spacing.
    // Larger values allow more misalignment between wheel and linear segment grousers.
    const double phaseAlignmentThreshold = 0.005;

    int adjustedNumGrousers = spec_->numShoes;
    {
        auto isPhaseAligned = [&](int n) -> bool {
            if(n < 4) return false;
            double spacing = trackCircumference / n;
            double segmentRemainder = fmod(wheelDistance, spacing);
            double segmentError = std::min(segmentRemainder, spacing - segmentRemainder);
            return segmentError < spacing * phaseAlignmentThreshold;
        };

        if(!isPhaseAligned(spec_->numShoes)){
            int adjusted = spec_->numShoes;
            for(int delta = 1; delta <= spec_->numShoes / 2; ++delta){
                if(isPhaseAligned(spec_->numShoes + delta)){
                    adjusted = spec_->numShoes + delta;
                    break;
                }
                if(isPhaseAligned(spec_->numShoes - delta)){
                    adjusted = spec_->numShoes - delta;
                    break;
                }
            }
            if(adjusted != spec_->numShoes){
                std::string suggestion;
                double adjustedSpacing = trackCircumference / adjusted;
                int k = static_cast<int>(std::round(wheelDistance / adjustedSpacing));
                if(k > 0){
                    double idealWheelDistance = k * adjustedSpacing;
                    suggestion += fmt::format(
                        " Recommended wheel distance: {:.4f}.", idealWheelDistance);
                    if(effectiveSprocketRadius == effectiveIdlerRadius && k > 0){
                        double idealRadius =
                            wheelDistance * (adjusted - 2 * k) / (2.0 * k * M_PI)
                            - spec_->thickness;
                        if(idealRadius > 0.0){
                            suggestion += fmt::format(
                                " Recommended wheel radius: {:.4f}.", idealRadius);
                        }
                    }
                }
                MessageOut::master()->putWarningln(
                    formatR(_("PxContinuousTrack \"{0}\" (link \"{1}\"): "
                              "Adjusted num_shoes from {2} to {3} "
                              "for phase alignment between wheels and linear segments.{4}"),
                            name(), trackLink->name(),
                            spec_->numShoes, adjusted, suggestion));
                adjustedNumGrousers = adjusted;
                grouserSpacing = trackCircumference / adjusted;
            } else {
                // Not adjustable - report error
                double wheelArcSum = M_PI * (effectiveSprocketRadius + effectiveIdlerRadius);
                int n = spec_->numShoes;
                double bestBelow = 0.0;
                double bestAbove = 0.0;
                for(int k = 1; k < n / 2; ++k){
                    double d = k * wheelArcSum / (n - 2 * k);
                    if(d <= wheelDistance && d > bestBelow){
                        bestBelow = d;
                    }
                    if(d >= wheelDistance && (bestAbove == 0.0 || d < bestAbove)){
                        bestAbove = d;
                    }
                }
                std::string suggestion;
                if(bestBelow > 0.0){
                    suggestion += fmt::format("{:.4f}", bestBelow);
                }
                if(bestAbove > 0.0 && bestAbove != bestBelow){
                    if(!suggestion.empty()) suggestion += " or ";
                    suggestion += fmt::format("{:.4f}", bestAbove);
                }
                std::string radiusSuggestion;
                if(effectiveSprocketRadius == effectiveIdlerRadius){
                    // When both wheel radii are equal, suggest ideal radius
                    auto suggestRadius = [&](double d) -> std::string {
                        int k = static_cast<int>(std::round(d / (trackCircumference / n)));
                        if(k > 0 && n > 2 * k){
                            double r = d * (n - 2 * k) / (2.0 * k * M_PI)
                                       - spec_->thickness;
                            if(r > 0.0){
                                return fmt::format("{:.4f}", r);
                            }
                        }
                        return {};
                    };
                    std::string radii;
                    if(bestBelow > 0.0){
                        auto r = suggestRadius(bestBelow);
                        if(!r.empty()) radii = r;
                    }
                    if(bestAbove > 0.0 && bestAbove != bestBelow){
                        auto r = suggestRadius(bestAbove);
                        if(!r.empty()){
                            if(!radii.empty()) radii += " or ";
                            radii += r;
                        }
                    }
                    if(!radii.empty()){
                        radiusSuggestion = fmt::format(
                            " Suggested wheel radius: {}", radii);
                    }
                }
                MessageOut::master()->putErrorln(
                    formatR(_("PxContinuousTrack \"{0}\" (link \"{1}\"): "
                              "num_shoes={2} is not phase-aligned "
                              "with wheel distance {3:.4f}. "
                              "Suggested wheel distance: {4}.{5}"),
                            name(), trackLink->name(),
                            spec_->numShoes, wheelDistance,
                            suggestion, radiusSuggestion));
                // geometryInitialized remains false, skip shoe positions
                return;
            }
        }
    }

    // Number of grousers on each linear segment
    int numGrousersPerSegment = static_cast<int>(floor(wheelDistance / grouserSpacing));
    if(numGrousersPerSegment < 1) numGrousersPerSegment = 1;

    // Number of grousers around each wheel (full circle for physics)
    int sprocketFullGrousers = static_cast<int>(round(2.0 * M_PI * effectiveSprocketRadius / grouserSpacing));
    if(sprocketFullGrousers < 4) sprocketFullGrousers = 4;
    int idlerFullGrousers = static_cast<int>(round(2.0 * M_PI * effectiveIdlerRadius / grouserSpacing));
    if(idlerFullGrousers < 4) idlerFullGrousers = 4;

    // Number of visible grousers on each wheel (outer half-arc including boundary).
    // The +1 accounts for a boundary grouser near the segment-wheel transition.
    int numGrousersOnSprocket = sprocketFullGrousers / 2 + 1;
    int numGrousersOnIdler = idlerFullGrousers / 2 + 1;

    Vector3 slideDir = diff.normalized();

    Vector3 upperCenter = (sprocketPos + idlerPos) / 2.0;
    upperCenter.z() += avgWheelRadius;

    Vector3 lowerCenter = (sprocketPos + idlerPos) / 2.0;
    lowerCenter.z() -= avgWheelRadius;

    // Recount grousers per segment using the same logic as createLinearSegment
    int upperGrousers = 0;
    {
        double x = wheelDistance / 2.0 - grouserSpacing;
        while(x >= -wheelDistance / 2.0 - grouserSpacing * 0.01){
            upperGrousers++;
            x -= grouserSpacing;
        }
    }
    int lowerGrousers = upperGrousers; // Same length segments

    // Store computed geometry in Spec
    spec_->wheelDistance = wheelDistance;
    spec_->grouserSpacing = grouserSpacing;
    spec_->avgWheelRadius = avgWheelRadius;
    spec_->effectiveSprocketRadius = effectiveSprocketRadius;
    spec_->effectiveIdlerRadius = effectiveIdlerRadius;
    spec_->numGrousersPerSegment = numGrousersPerSegment;
    spec_->numGrousersOnSprocket = numGrousersOnSprocket;
    spec_->numGrousersOnIdler = numGrousersOnIdler;
    spec_->upperSegmentCenter = upperCenter;
    spec_->lowerSegmentCenter = lowerCenter;
    spec_->slideDir = slideDir;
    spec_->segmentLength = wheelDistance;
    spec_->sprocketFullGrousers = sprocketFullGrousers;
    spec_->idlerFullGrousers = idlerFullGrousers;
    spec_->angleStepSprocket = 2.0 * M_PI / sprocketFullGrousers;
    spec_->angleStepIdler = 2.0 * M_PI / idlerFullGrousers;

    const int targetBeltN = 48;
    int sprocketMult = std::max(1, static_cast<int>(
        round(static_cast<double>(targetBeltN) / sprocketFullGrousers)));
    spec_->sprocketBeltSegments = sprocketFullGrousers * sprocketMult;
    spec_->angleStepSprocketBelt = 2.0 * M_PI / spec_->sprocketBeltSegments;

    int idlerMult = std::max(1, static_cast<int>(
        round(static_cast<double>(targetBeltN) / idlerFullGrousers)));
    spec_->idlerBeltSegments = idlerFullGrousers * idlerMult;
    spec_->angleStepIdlerBelt = 2.0 * M_PI / spec_->idlerBeltSegments;

    // Upper bound on visible shoe count, used for fixed-size state I/O buffer.
    // The actual count may be slightly less because wheel rotation shifts
    // the half-arc visibility boundary for grousers on each wheel.
    spec_->maxNumVisibleShoes = upperGrousers + lowerGrousers + numGrousersOnSprocket + numGrousersOnIdler;

    // Compute initial shoe positions (joint position = 0)
    shoePositions_.clear();
    shoePositions_.reserve(spec_->maxNumVisibleShoes);

    double shoeHeight = spec_->grouserHeight;

    // Upper segment grousers (direction = +1, grousers point up)
    for(int i = 0; i < upperGrousers; ++i){
        double x = wheelDistance / 2.0 - grouserSpacing - i * grouserSpacing;
        Vector3 pos = upperCenter + slideDir * x;
        pos.z() += spec_->thickness + shoeHeight / 2.0;
        Quaternion rot = Quaternion::Identity();
        shoePositions_.push_back(SE3(pos, rot));
    }

    // Lower segment grousers (direction = -1, grousers point down)
    for(int i = 0; i < lowerGrousers; ++i){
        double x = wheelDistance / 2.0 - grouserSpacing - i * grouserSpacing;
        Vector3 pos = lowerCenter + slideDir * x;
        pos.z() -= spec_->thickness + shoeHeight / 2.0;
        Quaternion rot = Quaternion::Identity();
        shoePositions_.push_back(SE3(pos, rot));
    }

    // Sprocket grousers (outer half-arc, cos(theta) >= 0)
    {
        Vector3 wheelCenter = sprocketPos;
        wheelCenter.z() = sprocketPos.z(); // wheel center in track local frame
        double angleStep = spec_->angleStepSprocket;
        for(int i = 0; i < sprocketFullGrousers; ++i){
            double theta = M_PI / 2.0 + i * angleStep;
            double c = cos(theta);
            if(c >= -0.01){
                double r = sprocketRadius + spec_->thickness + shoeHeight / 2.0;
                double lx = r * cos(theta);
                double lz = r * sin(theta);
                Vector3 pos = wheelCenter;
                pos += slideDir * lx;
                pos.z() += lz;
                // Grouser orientation: radial outward from wheel center
                // Rotate around Y axis so that the grouser's Z axis points radially
                Quaternion rot(Eigen::AngleAxisd(M_PI / 2.0 - theta, Vector3::UnitY()));
                shoePositions_.push_back(SE3(pos, rot));
            }
        }
    }

    // Idler grousers (outer half-arc, cos(theta) <= 0)
    {
        Vector3 wheelCenter = idlerPos;
        double angleStep = spec_->angleStepIdler;
        for(int i = 0; i < idlerFullGrousers; ++i){
            double theta = M_PI / 2.0 + i * angleStep;
            double c = cos(theta);
            if(c <= 0.01){
                double r = idlerRadius + spec_->thickness + shoeHeight / 2.0;
                double lx = r * cos(theta);
                double lz = r * sin(theta);
                Vector3 pos = wheelCenter;
                pos += slideDir * lx;
                pos.z() += lz;
                Quaternion rot(Eigen::AngleAxisd(M_PI / 2.0 - theta, Vector3::UnitY()));
                shoePositions_.push_back(SE3(pos, rot));
            }
        }
    }

    spec_->geometryInitialized = true;
    notifyStateChange();
}


int PxContinuousTrack::stateSize() const
{
    // The actual number of visible shoes varies per frame due to wheel rotation
    // shifting the half-arc boundary, but the state buffer size must be fixed.
    // Use maxNumVisibleShoes (the upper bound) with an extra slot for the actual count.
    // Layout: [actualCount, shoe0(7), shoe1(7), ..., padding(7)...]
    if(!spec_) return 0;
    return 1 + spec_->maxNumVisibleShoes * 7;
}


const double* PxContinuousTrack::readState(const double* buf, int size)
{
    // Read only the actual shoe count (not the padded buffer size)
    int actualCount = static_cast<int>(buf[0]);
    int maxCount = (size - 1) / 7;
    if(actualCount > maxCount){
        actualCount = maxCount;
    }
    shoePositions_.resize(actualCount);
    int i = 1;
    for(int j = 0; j < actualCount; ++j){
        SE3& s = shoePositions_[j];
        s.translation() << buf[i], buf[i+1], buf[i+2];
        i += 3;
        s.rotation() = Quaternion(buf[i], buf[i+1], buf[i+2], buf[i+3]);
        i += 4;
    }
    // Skip remaining padding to advance past the fixed-size buffer
    return buf + size;
}


double* PxContinuousTrack::writeState(double* out_buf) const
{
    int totalSize = spec_ ? (1 + spec_->maxNumVisibleShoes * 7) : 0;
    if(totalSize == 0){
        return out_buf;
    }
    // Store the actual visible shoe count first
    out_buf[0] = static_cast<double>(shoePositions_.size());
    int i = 1;
    for(size_t j = 0; j < shoePositions_.size(); ++j){
        const SE3& s = shoePositions_[j];
        auto& p = s.translation();
        out_buf[i++] = p.x();
        out_buf[i++] = p.y();
        out_buf[i++] = p.z();
        auto& q = s.rotation();
        out_buf[i++] = q.w();
        out_buf[i++] = q.x();
        out_buf[i++] = q.y();
        out_buf[i++] = q.z();
    }
    // Zero-fill unused slots to maintain the fixed buffer size
    std::fill(out_buf + i, out_buf + totalSize, 0.0);
    return out_buf + totalSize;
}


void PxContinuousTrack::addShoePosition(const SE3& pos)
{
    shoePositions_.push_back(pos);
}


void PxContinuousTrack::clearShoePositions()
{
    shoePositions_.clear();
}

}
