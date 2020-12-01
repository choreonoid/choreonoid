#ifndef CNOID_BODY_PLUGIN_LINK_SHAPE_OVERWRITE_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_SHAPE_OVERWRITE_ITEM_H

#include "BodyElementOverwriteItem.h"
#include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class Link;
class SgShape;

class CNOID_EXPORT LinkShapeOverwriteItem : public BodyElementOverwriteItem,
                                            public LocatableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    LinkShapeOverwriteItem();
    LinkShapeOverwriteItem(const LinkShapeOverwriteItem& org);
    virtual ~LinkShapeOverwriteItem();

    /**
       \note Currently the following function works correctly when
       the BodyOverwriteAddon::checkIfSingleShapeBody function returns true.
    */
    bool overwriteLinkShape(BodyItem* bodyItem, Link* link);
    
    Link* link();

    SgShape* shapeNode();
    void setShapeNode(SgShape* shape);

    Isometry3 shapeOffset() const;
    void setShapeOffset(const Isometry3& T);

    void cancelOverwriting();

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    // RenderableItem function. This returns the link origin marker.
    virtual SgNode* getScene() override;

    class Impl;

protected:
    virtual Item* doDuplicate(Item* duplicatedParentItem) const override;
    void onDisconnectedFromBodyItem() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    Impl* impl;

    class FileIO;
};

typedef ref_ptr<LinkShapeOverwriteItem> LinkShapeOverwriteItemPtr;

}

#endif
