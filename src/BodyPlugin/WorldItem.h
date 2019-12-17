/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_WORLD_ITEM_H
#define CNOID_BODY_PLUGIN_WORLD_ITEM_H

#include "BodyItem.h"
#include <cnoid/Item>
#include <cnoid/ItemList>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class WorldItemImpl;
class CollisionDetector;
class MaterialTable;

class CNOID_EXPORT WorldItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldItem();
    WorldItem(const WorldItem& org);
    virtual ~WorldItem();

    ItemList<BodyItem> coldetBodyItems() const;

    bool selectCollisionDetector(const std::string& name);
    CollisionDetector* collisionDetector();
    void enableCollisionDetection(bool on);
    bool isCollisionDetectionEnabled();
    void updateCollisionDetectorLater();
    void updateCollisionDetector();
    void updateCollisions();
    std::vector<CollisionLinkPairPtr>& collisions() const;
    SignalProxy<void()> sigCollisionsUpdated();

    // RenderableItem
    virtual SgNode* getScene() override;

    void setMaterialTableFile(const std::string& filename);
    MaterialTable* materialTable(bool checkFileUpdate = true);

    //! \deprecated
    ItemList<BodyItem> collisionBodyItems() const { return coldetBodyItems(); }

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    WorldItemImpl* impl;
};

typedef ref_ptr<WorldItem> WorldItemPtr;

}

#endif
