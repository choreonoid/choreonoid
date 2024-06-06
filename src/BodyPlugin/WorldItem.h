#ifndef CNOID_BODY_PLUGIN_WORLD_ITEM_H
#define CNOID_BODY_PLUGIN_WORLD_ITEM_H

#include "BodyItem.h"
#include <cnoid/Item>
#include <cnoid/ItemList>
#include <cnoid/RenderableItem>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CollisionDetector;
class MaterialTable;
class MaterialTableItem;

class CNOID_EXPORT WorldItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    WorldItem();
    virtual ~WorldItem();

    void storeCurrentBodyPositionsAsInitialPositions();
    void restoreInitialBodyPositions(bool doNotify = true);

    ItemList<BodyItem> coldetBodyItems() const;

    bool selectCollisionDetector(const std::string& name);
    CollisionDetector* collisionDetector();
    void setCollisionDetectionEnabled(bool on);
    bool isCollisionDetectionEnabled();
    void setCollisionDetectionBetweenMultiplexBodiesEnabled(bool on);
    bool isCollisionDetectionBetweenMultiplexBodiesEnabled() const;
    
    void updateCollisionDetectionBodies();
    void updateCollisionDetectionBodiesLater();
    void updateCollisions();

    typedef std::shared_ptr<CollisionLinkPair> CollisionLinkPairPtr;
    std::vector<CollisionLinkPairPtr>& collisions() const;
    SignalProxy<void()> sigCollisionsUpdated();

    void setDefaultMaterialTableFile(const std::string& filename);
    MaterialTable* defaultMaterialTable(bool checkFileUpdate = true);
    MaterialTable* materialTable();

    // RenderableItem
    virtual SgNode* getScene() override;


    [[deprecated("Use coldetBodyItems.")]]
    ItemList<BodyItem> collisionBodyItems() const { return coldetBodyItems(); }

    [[deprecated("Use setCollisionDetectionEnabled.")]]
    void enableCollisionDetection(bool on){ setCollisionDetectionEnabled(on); }

    [[deprecated("Use updateCollisionDetectionBodies.")]]
    void updateCollisionDetector() { updateCollisionDetectionBodies(); }

    [[deprecated("Use updateCollisionDetectionBodiesLater.")]]
    void updateCollisionDetectorLater() { updateCollisionDetectionBodiesLater(); }
    
protected:
    WorldItem(const WorldItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<WorldItem> WorldItemPtr;

}

#endif
