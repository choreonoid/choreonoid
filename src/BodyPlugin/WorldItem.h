/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_WORLD_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_WORLD_ITEM_H_INCLUDED

#include "BodyItem.h"
#include <cnoid/Item>
#include <cnoid/ItemList>
#include <cnoid/SceneProvider>
#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class WorldItemImpl;

class CNOID_EXPORT WorldItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldItem();
    WorldItem(const WorldItem& org);
    virtual ~WorldItem();

    const ItemList<BodyItem>& bodyItems() const;

    bool selectCollisionDetector(const std::string& name);
    CollisionDetectorPtr collisionDetector();
    void enableCollisionDetection(bool on);
    bool isCollisionDetectionEnabled();
    void updateCollisionDetector();
    void updateCollisions();
    const std::vector<CollisionLinkPairPtr>& collisions() const;
    SignalProxy< boost::signal<void()> > sigCollisionsUpdated();

    virtual SgNode* getScene();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    WorldItemImpl* impl;
};

typedef ref_ptr<WorldItem> WorldItemPtr;
}

#endif

