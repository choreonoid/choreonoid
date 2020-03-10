#ifndef CNOID_BODY_PLUGIN_LINK_SHAPE_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_SHAPE_ITEM_H

#include <cnoid/Item>
#include <cnoid/LocatableItem>
#include "exportdecl.h"

namespace cnoid {

class SgNode;

class CNOID_EXPORT LinkShapeItem : public Item, public LocatableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    LinkShapeItem();
    LinkShapeItem(const LinkShapeItem& org);
    virtual ~LinkShapeItem();

    void setShape(SgNode* shape);

    Position shapeOffset() const;
    void setShapeOffset(const Position& T);

    // LocatableItem functions
    virtual Position getLocation() const override;
    virtual bool prefersLocalLocation() const override;
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual void setLocation(const Position& T) override;
    virtual LocatableItem* getParentLocatableItem() override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;

    class FileIO;
};

typedef ref_ptr<LinkShapeItem> LinkShapeItemPtr;

}

#endif

