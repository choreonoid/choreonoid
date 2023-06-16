#ifndef CNOID_MOCAP_PLUGIN_SKELETON_MOTION_ITEM_H
#define CNOID_MOCAP_PLUGIN_SKELETON_MOTION_ITEM_H

#include "SkeletonMotion.h"
#include <cnoid/MultiValueSeqItem>
#include <cnoid/RenderableItem>
#include <cnoid/LocatableItem>
#include <cnoid/ConnectionSet>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SkeletonMotionItem : public MultiValueSeqItem, public RenderableItem, public LocatableItem
{
public:

    static void initialize(ExtensionManager* ext);
    static void loadAMCfiles();

    SkeletonMotionItem();
    ~SkeletonMotionItem();

    SkeletonMotionPtr motion() { return motion_; }
    void resetMotion(SkeletonMotionPtr motion);

    bool loadBVH(const std::string& filename, std::ostream& os);
    bool saveBVH(const std::string& filename, std::ostream& os);

    virtual SgNode* getScene() override;
    virtual LocationProxyPtr getLocationProxy() override;

    class SceneSkeletonMotion;
    class Location;

protected:
    SkeletonMotionItem(const SkeletonMotionItem& org);

    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    SkeletonMotionPtr motion_;
    ref_ptr<SceneSkeletonMotion> scene_;
    ref_ptr<Location> location_;
    ConnectionSet connectionsInCheckState;
    ConnectionSet connectionsInCheckStateGL;

    void createScene();
    void onSceneConnection(bool on);
};

typedef ref_ptr<SkeletonMotionItem> SkeletonMotionItemPtr;

}

#endif
