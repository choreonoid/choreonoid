/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_WORLD_ITEM_H
#define CNOID_BODY_PLUGIN_WORLD_ITEM_H

#include "BodyItem.h"
#include <cnoid/Item>
#include <cnoid/ItemList>
#include <cnoid/SceneProvider>
#include <cnoid/CollisionDetector>
#include "exportdecl.h"

namespace cnoid {

class WorldItemImpl;
class ContactMaterialTable;

class CNOID_EXPORT WorldItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldItem();
    WorldItem(const WorldItem& org);
    virtual ~WorldItem();

    const ItemList<BodyItem>& collisionBodyItems() const;

    bool selectCollisionDetector(const std::string& name);
    CollisionDetectorPtr collisionDetector();
    void enableCollisionDetection(bool on);
    bool isCollisionDetectionEnabled();
    void updateCollisionDetectorLater();
    void updateCollisionDetector();
    void updateCollisions();
    std::vector<CollisionLinkPairPtr>& collisions() const;
    SignalProxy<void()> sigCollisionsUpdated();

    virtual SgNode* getScene();

    void setContactMaterialFile(const std::string& filename);
    ContactMaterialTable* contactMaterialTable();

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    WorldItemImpl* impl;
};

typedef ref_ptr<WorldItem> WorldItemPtr;

}

#endif
