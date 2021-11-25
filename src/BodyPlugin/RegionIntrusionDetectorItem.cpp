#include "RegionIntrusionDetectorItem.h"
#include <cnoid/ItemManager>
#include <cnoid/AISTCollisionDetector>
#include <cnoid/Body>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/DigitalIoDevice>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneDrawables>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/EigenArchive>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class RegionLocation : public LocationProxy
{
public:
    RegionIntrusionDetectorItem::Impl* impl;
    Signal<void()> sigLocationChanged_;

    RegionLocation(RegionIntrusionDetectorItem::Impl* impl);
    virtual Item* getCorrespondingItem() override;
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace cnoid {

class RegionIntrusionDetectorItem::Impl
{
public:
    RegionIntrusionDetectorItem* self;
    unique_ptr<AISTCollisionDetector> collisionDetector;
    unique_ptr<BodyCollisionDetector> bodyCollisionDetector;
    DigitalIoDevicePtr ioDevice;
    int ioSignalNumber;
    bool isIntruding;
    bool intrusionChanged;

    // Region model members for the collision detection
    Vector3 boxRegionSize;
    Isometry3 regionOffset;
    BodyPtr regionBody;
    LinkPtr regionLink;
    MeshGenerator meshGenerator;
    SgPosTransformPtr regionShapeLocalOffset;
    SgShapePtr regionShape;

    ref_ptr<RegionLocation> regionLocation;

    // Region model members for the visualization
    SgPosTransformPtr markerTransform;
    SgLineSetPtr markerLineSet;
    SgVertexArrayPtr markerVertices;
    
    Impl(RegionIntrusionDetectorItem* self);
    Impl(RegionIntrusionDetectorItem* self, Impl& org);
    void initializeCollisionDetector();
    bool initialize(ControllerIO* io);
    void createRegionMarker();
    void updateMarkerVertices();
    void updateMarkerPosition();
};

}


void RegionIntrusionDetectorItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<RegionIntrusionDetectorItem, ControllerItem>(N_("RegionIntrusionDetector"))
        .addCreationPanel<RegionIntrusionDetectorItem>();
}


RegionIntrusionDetectorItem::RegionIntrusionDetectorItem()
{
    impl = new Impl(this);
}


RegionIntrusionDetectorItem::RegionIntrusionDetectorItem(const RegionIntrusionDetectorItem& org)
    : ControllerItem(org)
{
    impl = new Impl(this, *org.impl);
}


RegionIntrusionDetectorItem::Impl::Impl(RegionIntrusionDetectorItem* self)
    : self(self)
{
    boxRegionSize.setOnes();
    regionOffset.setIdentity();
    ioSignalNumber = 0;
}


RegionIntrusionDetectorItem::Impl::Impl(RegionIntrusionDetectorItem* self, Impl& org)
    : Impl(self)
{
    boxRegionSize = org.boxRegionSize;
    regionOffset = org.regionOffset;
    ioSignalNumber = org.ioSignalNumber;
}


RegionIntrusionDetectorItem::~RegionIntrusionDetectorItem()
{
    delete impl;
}


Item* RegionIntrusionDetectorItem::doDuplicate() const
{
    return new RegionIntrusionDetectorItem(*this);
}


bool RegionIntrusionDetectorItem::setBoxRegionSize(const Vector3& size)
{
    if(size.minCoeff() > 0.0){ // (size.array() > 0.0).all()
        impl->boxRegionSize = size;
        impl->updateMarkerVertices();
        return true;
    }
    return false;
}


const Vector3& RegionIntrusionDetectorItem::boxRegionSize() const
{
    return impl->boxRegionSize;
}


void RegionIntrusionDetectorItem::setRegionOffset(const Isometry3& T)
{
    impl->regionOffset = T;
    impl->updateMarkerPosition();
    if(impl->regionLocation){
        impl->regionLocation->sigLocationChanged_();
    }
}


const Isometry3& RegionIntrusionDetectorItem::regionOffset() const
{
    return impl->regionOffset;
}


void RegionIntrusionDetectorItem::setDigitalIoSignalNumber(int no)
{
    impl->ioSignalNumber = no;
}


int RegionIntrusionDetectorItem::digitalIoSignalNumber() const
{
    return impl->ioSignalNumber;
}


void RegionIntrusionDetectorItem::Impl::initializeCollisionDetector()
{
    collisionDetector.reset(new AISTCollisionDetector);
    bodyCollisionDetector.reset(new BodyCollisionDetector);
    bodyCollisionDetector->setCollisionDetector(collisionDetector.get());

    regionBody = new Body;
    regionLink = regionBody->rootLink();
    regionLink->setJointType(Link::FixedJoint);
    regionShapeLocalOffset = new SgPosTransform;
    regionShape = new SgShape;
    regionShapeLocalOffset->addChild(regionShape);
    regionLink->addCollisionShapeNode(regionShapeLocalOffset);
}    


bool RegionIntrusionDetectorItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool RegionIntrusionDetectorItem::Impl::initialize(ControllerIO* io)
{
    if(!collisionDetector){
        initializeCollisionDetector();
    }
    
    auto body = io->body();
    ioDevice = body->findDevice<DigitalIoDevice>();
    if(!ioDevice){
        io->os() << format(_("\"{0}\" cannot work with \"{1}\" because it does not have a digital IO device."),
                           self->name(), body->name()) << endl;
        return false;
    }

    bodyCollisionDetector->clearBodies();

    regionShapeLocalOffset->setTranslation(Vector3(0.0, 0.0, boxRegionSize.z() / 2.0));
    regionShape->setMesh(meshGenerator.generateBox(boxRegionSize));
    regionLink->setPosition(regionOffset);
    bodyCollisionDetector->addBody(regionBody, false);

    bodyCollisionDetector->addBody(io->body(), false);
    bodyCollisionDetector->makeReady();

    isIntruding = false;
    intrusionChanged = false;

    return true;
}


void RegionIntrusionDetectorItem::input()
{
    impl->bodyCollisionDetector->updatePositions();
}


bool RegionIntrusionDetectorItem::control()
{
    bool prevIntrusion = impl->isIntruding;
    impl->isIntruding = false;
    impl->bodyCollisionDetector->detectCollisions(
        [this](const CollisionPair&){ impl->isIntruding = true; });
    impl->intrusionChanged = (impl->isIntruding != prevIntrusion);
    return false;
}


void RegionIntrusionDetectorItem::output()
{
    if(impl->intrusionChanged){
        impl->ioDevice->setOut(impl->ioSignalNumber, impl->isIntruding, true);
        impl->intrusionChanged = false;
    }
}


void RegionIntrusionDetectorItem::stop()
{
    impl->ioDevice.reset();
    impl->bodyCollisionDetector->clearBodies();
}


LocationProxyPtr RegionIntrusionDetectorItem::getLocationProxy()
{
    if(!impl->regionLocation){
        impl->regionLocation = new RegionLocation(impl);
    }
    return impl->regionLocation;
}


RegionLocation::RegionLocation(RegionIntrusionDetectorItem::Impl* impl)
    : LocationProxy(GlobalLocation),
      impl(impl)
{

}


Item* RegionLocation::getCorrespondingItem()
{
    return impl->self;
}


Isometry3 RegionLocation::getLocation() const
{
    return impl->regionOffset;
}


bool RegionLocation::setLocation(const Isometry3& T)
{
    impl->self->setRegionOffset(T);
    impl->self->notifyUpdate();
    return true;
}


SignalProxy<void()> RegionLocation::sigLocationChanged()
{
    return sigLocationChanged_;
}


SgNode* RegionIntrusionDetectorItem::getScene()
{
    if(!impl->markerTransform){
        impl->createRegionMarker();
    }
    return impl->markerTransform;
}


void RegionIntrusionDetectorItem::Impl::createRegionMarker()
{
    markerVertices = new SgVertexArray;
    updateMarkerVertices();
    
    markerLineSet = new SgLineSet;
    markerLineSet->setVertices(markerVertices);
    markerLineSet->setLineWidth(2.0f);
    markerLineSet->setNumLines(12);
    markerLineSet->setLine(0, 0, 1);
    markerLineSet->setLine(1, 1, 3);
    markerLineSet->setLine(2, 3, 2);
    markerLineSet->setLine(3, 2, 0);
    markerLineSet->setLine(4, 4, 5);
    markerLineSet->setLine(5, 5, 7);
    markerLineSet->setLine(6, 7, 6);
    markerLineSet->setLine(7, 6, 4);
    markerLineSet->setLine(8, 0, 4);
    markerLineSet->setLine(9, 1, 5);
    markerLineSet->setLine(10, 3, 7);
    markerLineSet->setLine(11, 2, 6);

    auto material = markerLineSet->getOrCreateMaterial();
    material->setDiffuseColor(Vector3f(1.0f, 0.0f, 0.0f));

    markerTransform = new SgPosTransform(regionOffset);
    markerTransform->addChild(markerLineSet);
}


void RegionIntrusionDetectorItem::Impl::updateMarkerVertices()
{
    if(markerVertices){
        auto& v = *markerVertices;
        float x = boxRegionSize.x() / 2.0;
        float y = boxRegionSize.y() / 2.0;
        float z = boxRegionSize.z();
        v.resize(8);
        v[0] << -x, -y, 0.0;
        v[1] << -x, -y, z;
        v[2] << -x,  y, 0.0;
        v[3] << -x,  y, z;
        v[4] <<  x, -y, 0.0;
        v[5] <<  x, -y, z;
        v[6] <<  x,  y, 0.0;
        v[7] <<  x,  y, z;
        markerVertices->notifyUpdate();
    }
}


void RegionIntrusionDetectorItem::Impl::updateMarkerPosition()
{
    if(markerTransform){
        markerTransform->setPosition(regionOffset);
        markerTransform->notifyUpdate();
    }
}


void RegionIntrusionDetectorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Box region size"), str(impl->boxRegionSize),
                [&](const string& str){
                    Vector3 s;
                    if(toVector3(str, s)){
                        return setBoxRegionSize(s);
                    }
                    return false;
                });

    putProperty(_("Region offset"), str(Vector3(impl->regionOffset.translation())),
                [&](const string& str){
                    Vector3 p;
                    if(toVector3(str, p)){
                        impl->regionOffset.translation() = p;
                        setRegionOffset(impl->regionOffset);
                        return true;
                    }
                    return false;
                });

    auto rpy = rpyFromRot(impl->regionOffset.linear());
    putProperty(_("Region yaw angle"), degree(rpy.z()),
                [&](double a){
                    impl->regionOffset.linear() = rotFromRpy(Vector3(0.0, 0.0, radian(a)));
                    setRegionOffset(impl->regionOffset);
                    return true;
                });

    putProperty.min(0)
        (_("I/O signal number"), impl->ioSignalNumber,
         [&](int no){ impl->ioSignalNumber = no; return true; });
}


bool RegionIntrusionDetectorItem::store(Archive& archive)
{
    write(archive, "box_region_size", impl->boxRegionSize);
    if(!impl->regionOffset.translation().isZero()){
        write(archive, "translation", impl->regionOffset.translation());
    }
    AngleAxis aa(impl->regionOffset.linear());
    if(aa.angle() != 0.0){
        writeDegreeAngleAxis(archive, "rotation", aa);
    }
    archive.write("io_signal_number", impl->ioSignalNumber);
    return true;
}


bool RegionIntrusionDetectorItem::restore(const Archive& archive)
{
    Vector3 p;
    if(read(archive, "box_region_size", impl->boxRegionSize)){
        impl->updateMarkerVertices();
    }
    bool offsetUpdated = false;
    if(read(archive, "translation", p)){
        impl->regionOffset.translation() = p;
        offsetUpdated = true;
    }
    AngleAxis aa;
    if(readDegreeAngleAxis(archive, "rotation", aa)){
        impl->regionOffset.linear() = aa.toRotationMatrix();
        offsetUpdated = true;
    }
    if(offsetUpdated){
        setRegionOffset(impl->regionOffset);
    }
    archive.read("io_signal_number", impl->ioSignalNumber);
    return true;
}
