#include "SkeletonMotionItem.h"
#include "Bone.h"
#include "AMCLoader.h"
#include <cnoid/ItemManager>
#include <cnoid/MainMenu>
#include <cnoid/RootItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/TimeBar>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <cnoid/Archive>
#include <cnoid/SceneWidgetEventHandler>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneDrawables>
#include <cnoid/MeshGenerator>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

void drawBone(Bone* bone);
void drawBoneCylinder(float x0, float y0, float z0, float x1, float y1, float z1);

class SceneBone : public SgPosTransform
{
public:
    SgScaleTransformPtr scale;

    SceneBone(SgShapePtr boneShape);
    void setPosition(const Vector3f& p0, const Vector3f& p1);
};

typedef ref_ptr<SceneBone> SceneBonePtr;

}

namespace cnoid {

class SkeletonMotionItem::SceneSkeletonMotion : public SgGroup, public SceneWidgetEventHandler
{
public:
    SgShapePtr boneShape;
    SgMaterialPtr boneMaterial;
    SkeletonMotionPtr motion;
    vector<SceneBonePtr> sceneBones;
    int frame;
    bool isDirty;

    SceneSkeletonMotion();
    void setMotion(SkeletonMotionPtr motion);
    void setColor(const Vector3f& color);
    Vector3f color() const;
    void update();
    bool setTime(double time);
    void render(SceneRenderer::NodeFunctionSet* renderingFunctions);
    void updateBonePositions();
    void updateBonePosition(Bone* bone, int& index);
    virtual bool onPointerMoveEvent(SceneWidgetEvent* event) override;
    virtual void onPointerLeaveEvent(SceneWidgetEvent* event) override;
};

class SkeletonMotionItem::Location : public LocationProxy
{
public:
    SkeletonMotionItem* item;

    Location(SkeletonMotionItem* item);
    virtual std::string getName() const override;
    virtual Isometry3 getLocation() const override;
    virtual bool setLocation(const Isometry3& T) override;
    virtual SignalProxy<void()> sigLocationChanged() override;
};

}

namespace {

struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SkeletonMotionItem::SceneSkeletonMotion, SgGroup>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto functions = renderer->renderingFunctions();
                functions->setFunction<SkeletonMotionItem::SceneSkeletonMotion>(
                    [functions](SgNode* node){
                        static_cast<SkeletonMotionItem::SceneSkeletonMotion*>(node)->render(functions);
                    });
            });
    }
} registration;

}


void SkeletonMotionItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){

        ItemManager& im = ext->itemManager();

        im.registerClass<SkeletonMotionItem>(N_("SkeletonMotionItem"));
        im.addLoaderAndSaver<SkeletonMotionItem>(
            _("BVH format mocap file"), "BVH", "bvh",
            [](SkeletonMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->loadBVH(filename, os); },
            [](SkeletonMotionItem* item, const std::string& filename, std::ostream& os, Item* parentItem){
                return item->saveBVH(filename, os); });

        initialized = true;
    }
}


/**
   \todo rewrite the following code to use the ItemManger::addFileHander() function,
   which should be implemented.
*/
void SkeletonMotionItem::loadAMCfiles()
{
    string asfFile = getOpenFileName(_("Load an Acclaim Skeleton (ASF) File"), "asf");
    if(!asfFile.empty()){
        AMCLoader loader;
        loader.enableCoordinateFlipping(true);
        SkeletonPtr skeleton = loader.loadASF(asfFile, mvout());
        if(skeleton){
            vector<string> amcFiles = getOpenFileNames(_("Load Acclaim Motion Capture (AMC) Files"), "amc");

            auto rootItem = RootItem::instance();
            Item* parentItem = rootItem->selectedItems<Item>().toSingle();
            if(!parentItem){
                parentItem = rootItem;
            }
            
            for(auto& file : amcFiles){
                SkeletonMotionItemPtr item = new SkeletonMotionItem;
                item->setName(stdx::filesystem::path(file).stem().string());
                SkeletonMotionPtr motion = loader.loadAMC(file, mvout());
                if(motion){
                    item->resetMotion(motion);
                    parentItem->addChildItem(item, true);
                    item->notifyUpdate();
                }
            }
        }
    }
}

    
SkeletonMotionItem::SkeletonMotionItem()
    : MultiValueSeqItem(make_shared<SkeletonMotion>())
{
    setAttribute(Reloadable);    
    motion_ = static_pointer_cast<SkeletonMotion>(seq());
    createScene();
}


SkeletonMotionItem::SkeletonMotionItem(const SkeletonMotionItem& org)
    : MultiValueSeqItem(org, make_shared<SkeletonMotion>(*org.motion_)),
      motion_(static_pointer_cast<SkeletonMotion>(seq()))
{
    createScene();
}


void SkeletonMotionItem::createScene()
{
    scene_ = new SceneSkeletonMotion();
    scene_->setMotion(motion_);
    scene_->sigGraphConnection().connect([&](bool on){ onSceneConnection(on); });
}


SkeletonMotionItem::~SkeletonMotionItem()
{
    if(location_){
        location_->expire();
    }
}


void SkeletonMotionItem::resetMotion(SkeletonMotionPtr motion)
{
    motion_ = motion;
    MultiValueSeqItem::resetSeq(motion);
    scene_->setMotion(motion);
    //notifyUpdate(); // This should be manually done
}


void SkeletonMotionItem::onSceneConnection(bool on)
{
    connectionsInCheckStateGL.disconnect();
    
    if(on){
        connectionsInCheckStateGL.add(
            TimeBar::instance()->sigTimeChanged().connect([&](double time){ return scene_->setTime(time); }));
        scene_->setTime(TimeBar::instance()->time());
        connectionsInCheckStateGL.add(sigUpdated().connect([&](){ scene_->update(); }));
    }
}
   

SgNode* SkeletonMotionItem::getScene()
{
    return scene_;
}


LocationProxyPtr SkeletonMotionItem::getLocationProxy()
{
    if(!location_){
        location_ = new Location(this);
    }
    return location_;
}


bool SkeletonMotionItem::loadBVH(const std::string& filename, std::ostream& os)
{
    bool loaded = motion()->loadBVH(filename);

    if(!motion()->messages().empty()){
        os << motion()->messages() << endl;
    }
    return loaded;
}


bool SkeletonMotionItem::saveBVH(const std::string& filename, std::ostream& os)
{
    bool saved = motion()->saveBVH(filename);
    if(saved){
        notifyUpdate();
    }
    return saved;
}


Item* SkeletonMotionItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new SkeletonMotionItem(*this);
}


void SkeletonMotionItem::doPutProperties(PutPropertyFunction& putProperty)
{
    MultiValueSeqItem::doPutProperties(putProperty);

    putProperty(_("Number of bones"), motion_->skeleton()->numBones());

    putProperty(_("Offset time frame"), motion_->offsetTimeFrame(),
                [&](int offset){
                    motion_->setOffsetTimeFrame(offset);
                    TimeBar::instance()->refresh();
                    return true;
                });

    putProperty(_("Translation"), str(motion_->translation()),
                [&](const string& value){
                    Vector3 p;
                    if(toVector3(value, p)){
                        motion_->setTranslation(p);
                        return true;
                    }
                    return false;
                });

    putProperty(_("Yaw rotation"), degree(motion_->yawRotation()),
                [&](double angle){
                    motion_->setYawRotation(radian(angle));
                    return true;
                });

    putProperty(_("Scale"), motion_->skeleton()->scale(),
                [&](double scale){
                    if(scale > 0.0){
                        motion_->skeleton()->setScale(scale);
                        return true;
                    }
                    return false;
                });

    putProperty(_("Color"), str(scene_->color()),
                [&](const string& value){
                    Vector3f color;
                    if(toVector3(value, color)){
                        scene_->setColor(color);
                        return true;
                    }
                    return false;
                });
}


bool SkeletonMotionItem::store(Archive& archive)
{
    if(MultiValueSeqItem::store(archive)){
        auto translation = motion_->translation();
        if(translation != Vector3::Zero()){
            write(archive, "translation", translation);
        }
        writeDegreeAngleAxis(archive, "rotation", AngleAxis(motion_->positionOffset().linear()));
        archive.write("scale", motion_->skeleton()->scale());
        write(archive, "color", scene_->color());
        return true;
    }
    return false;
}


bool SkeletonMotionItem::restore(const Archive& archive)
{
    if(MultiValueSeqItem::restore(archive)){

        bool hasOffset = false;
        Isometry3 T_offset = Isometry3::Identity();
        Vector3 v;
        if(read(archive, { "translation", "rootOffset" }, v)){
            T_offset.translation() = v;
            hasOffset = true;
        }
        AngleAxis aa;
        if(readDegreeAngleAxis(archive, "rotation", aa)){
            T_offset.linear() = aa.toRotationMatrix();
            hasOffset = true;
        }
        motion_->setPositionOffset(T_offset);

        double scale;
        if(archive.read("scale", scale)){
            motion_->skeleton()->setScale(scale);
        }
        Vector3 c;
        if(read(archive, "color", c)){
            scene_->setColor(c.cast<float>());
        }
        return true;
    }
    
    return false;
}


SkeletonMotionItem::SceneSkeletonMotion::SceneSkeletonMotion()
    : SgGroup(findPolymorphicId<SceneSkeletonMotion>())
{
    boneShape = new SgShape;
    MeshGenerator generator;
    boneShape->setMesh(generator.generateCylinder(0.05, 1.0));
    boneMaterial = boneShape->getOrCreateMaterial();
    setColor(Vector3f(1.0f, 0.0f, 0.0f));
    frame = -1;
    isDirty = true;
}


void SkeletonMotionItem::SceneSkeletonMotion::setMotion(SkeletonMotionPtr motion)
{
    this->motion = motion;
}


void SkeletonMotionItem::SceneSkeletonMotion::setColor(const Vector3f& color)
{
    boneMaterial->setDiffuseColor(color);
    boneMaterial->notifyUpdate();
}


Vector3f SkeletonMotionItem::SceneSkeletonMotion::color() const
{
    return boneMaterial->diffuseColor();
}


void SkeletonMotionItem::SceneSkeletonMotion::update()
{
    if(motion->setFrameToSkeleton(frame)){
        isDirty = true;
        notifyUpdate();
    }
}


bool SkeletonMotionItem::SceneSkeletonMotion::setTime(double time)
{
    int numFrames = motion->numFrames();
    int newFrame = motion->frameOfTime(time);
    bool isValid = (newFrame < numFrames);
    if(!isValid){
        newFrame = numFrames - 1;
    }
    if(newFrame != frame){
        frame = newFrame;
        update();
    }
    return isValid;
}


void SkeletonMotionItem::SceneSkeletonMotion::render(SceneRenderer::NodeFunctionSet* renderingFunctions)
{
    if(isDirty){
        updateBonePositions();
        isDirty = false;
    }
    renderingFunctions->dispatchAs<SgGroup>(this);
}


void SkeletonMotionItem::SceneSkeletonMotion::updateBonePositions()
{
    Bone* rootBone = motion->skeleton()->rootBone();
    if(rootBone){
        clearChildren();
        int index = 0;
        updateBonePosition(rootBone, index);
    }
}


void SkeletonMotionItem::SceneSkeletonMotion::updateBonePosition(Bone* bone, int& index)
{
    auto parentBone = bone->parent();
    if(parentBone){
        if(index == sceneBones.size()){
            auto boneNode = new SceneBone(boneShape);
            boneNode->setName(fmt::format("{0} --> {1}", parentBone->name(), bone->name()));
            sceneBones.push_back(boneNode);
        }
        SceneBone* sceneBone = sceneBones[index];
        Vector3 o = parentBone->translation();
        Vector3 e = bone->translation();
        sceneBone->setPosition(Vector3f(o.x(), o.y(), o.z()), Vector3f(e.x(), e.y(), e.z()));
        addChild(sceneBone);
        ++index;
    }
    for(Bone* child = bone->child(); child; child = child->sibling()){
        updateBonePosition(child, index);
    }
}


bool SkeletonMotionItem::SceneSkeletonMotion::onPointerMoveEvent(SceneWidgetEvent* event)
{
    auto& path = event->nodePath();
    for(auto it = path.rbegin(); it != path.rend(); ++it){
        auto& node = *it;
        if(auto boneNode = dynamic_cast<SceneBone*>(node.get())){
            event->updateIndicator(fmt::format("Bone \"{0}\"", boneNode->name()));
            return true;
        }
    }
    return false;
}


void SkeletonMotionItem::SceneSkeletonMotion::onPointerLeaveEvent(SceneWidgetEvent* event)
{
    event->updateIndicator("");
}


SceneBone::SceneBone(SgShapePtr boneShape)
{
    scale = new SgScaleTransform;
    scale->addChild(boneShape);
    addChild(scale);
}


void SceneBone::setPosition(const Vector3f& p0, const Vector3f& p1)
{
    Vector3f direction = (p1 - p0);
    float length = direction.norm();
    scale->setScale(Vector3f(1.0f, length, 1.0f));
    
    Vector3f center = (p0 + p1) / 2.0f;
    setTranslation(center);
    
    Quaternionf q;
    q.setFromTwoVectors(Vector3f::UnitY(), direction.normalized());
    setRotation(q);
}


SkeletonMotionItem::Location::Location(SkeletonMotionItem* item)
    : LocationProxy(GlobalLocation),
      item(item)
{

}

std::string SkeletonMotionItem::Location::getName() const
{
    return fmt::format(_("Offset position of {0}"), item->displayName());
}


Isometry3 SkeletonMotionItem::Location::getLocation() const
{
    return item->motion_->positionOffset();
}


bool SkeletonMotionItem::Location::setLocation(const Isometry3& T)
{
    item->motion_->setPositionOffset(T);
    item->notifyUpdate();
    return true;
}


SignalProxy<void()> SkeletonMotionItem::Location::sigLocationChanged()
{
    return item->sigUpdated();
}
