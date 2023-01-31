#ifndef CNOID_BODY_PLUGIN_LINK_OVERWRITE_ITEM_H
#define CNOID_BODY_PLUGIN_LINK_OVERWRITE_ITEM_H

#include "BodyElementOverwriteItem.h"
#include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class Link;
class SgShape;

class CNOID_EXPORT LinkOverwriteItem : public BodyElementOverwriteItem,
                                       public LocatableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static LinkOverwriteItem* findLinkOverwriteItem(BodyItem* bodyItem, Link* link);

    LinkOverwriteItem();
    virtual ~LinkOverwriteItem();

    virtual bool setName(const std::string& name) override;

    enum OverwriteElement {
        NoElement          = 0,
        OffsetPosition     = 1 << 0,
        JointType          = 1 << 1,
        JointAxis          = 1 << 2,
        JointId            = 1 << 3,
        JointName          = 1 << 4,
        JointRange         = 1 << 5,
        JointVelocityRange = 1 << 6,
        JointEffortRange   = 1 << 7,
        Mass               = 1 << 8,
        Inertia            = 1 << 9,
        CenterOfMass       = 1 << 10,
        Material           = 1 << 11,
        Shape              = 1 << 12,
        AllElements =
          OffsetPosition | JointType | JointAxis | JointId | JointName | JointRange |
          JointVelocityRange | JointEffortRange | Mass | Inertia | CenterOfMass |
          Material | Shape,
    };

    void setTargetElementSet(int elementSet);
    void addTargetElement(int element);
    int targetElementSet() const;

    bool setReferenceLink(Link* referenceLink, bool isRootLink = false);
    Link* referenceLink();
    bool isRootLink() const;
    
    Link* originalLinkClone();
    
    bool setAdditionalLink(Link* additionalLink, const std::string& parentLinkName = std::string());
    Link* additionalLink();

    Link* sourceLink();
    Link* targetLink();

    bool isOverwriting() const;
    bool isOverwritingExistingLink() const;
    bool isAddingLink() const;

    bool updateOverwriting();
    virtual void releaseOverwriteTarget() override;

    SgPosTransform* shapeOffsetTransform();

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;

    enum LocationTargetType { LinkOffsetLocation, ShapeOffsetLocation };
    void setLocationTargetType(int type);
    int locationTargetType() const;

    // RenderableItem function. This returns the link origin marker.
    virtual SgNode* getScene() override;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    class Impl;

protected:
    LinkOverwriteItem(const LinkOverwriteItem& org, CloneMap* cloneMap);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual bool onNewOverwritePositionCheck(bool isManualOperation) override;
    virtual void onDisconnectedFromBodyItem() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    Impl* impl;
};

typedef ref_ptr<LinkOverwriteItem> LinkOverwriteItemPtr;

}

#endif
