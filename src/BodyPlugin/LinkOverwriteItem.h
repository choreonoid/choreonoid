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
        ShapeOffset        = 1 << 12,
        ShapeColor         = 1 << 13,
        Shape              = 1 << 14,
        AllNonShapeElements =
          OffsetPosition | JointType | JointAxis | JointId | JointName | JointRange |
          JointVelocityRange | JointEffortRange | Mass | Inertia | CenterOfMass | Material
    };

    void setOverwriteElementSet(int elementSet);
    void addOverwriteElement(int element);
    void addOverwriteElementSet(int elementSet);
    int overwriteElementSet() const;
    bool hasOverwriteElement(int element) const;

    [[deprecated("Use setOverwriteElementSet.")]]
    void setTargetElementSet(int elementSet) { setOverwriteElementSet(elementSet); }
    [[deprecated("Use addOverwriteElement.")]]
    void addTargetElement(int element) { addOverwriteElement(element); }
    [[deprecated("Use overwriteElementSet.")]]
    int targetElementSet() const { return overwriteElementSet(); }

    /**
       When a LinkOverwriteItem overwrites an existing link, the information of the link elements to be overwritten is
       stored in a reference link, which is a separate object from the existing link.
       Note that if the overwritten elements are only shape elements, the reference link is not necessary.
    */
    bool setReferenceLink(Link* referenceLink, bool isRootLink = false);
    Link* referenceLink();
    bool isRootLink() const;
    
    Link* originalLinkClone();
    
    /**
       When a LinkOverwriteItem creates a new link, the new link is stored as an "additional link".
       In this case, the LinkOverwriteItem does not have the reference link.
    */
    bool setAdditionalLink(Link* additionalLink, const std::string& parentLinkName = std::string());
    Link* additionalLink();

    void setShapeOffset(const Isometry3& T, bool doOverwrite = false);
    const Isometry3& shapeOffset() const;
    void setShapeColor(const Vector3f& color, bool doOverwrite = false);
    const Vector3f& shapeColor() const;
    void resetShapeColor(bool doNotify = false);

    void setShape(SgNode* shape);
    void setVisualShape(SgNode* shape);
    void setCollisionShape(SgNode* shape);
    SgNode* visualShape();
    SgNode* collisionShape();

    std::string findOriginalShapeFile() const;

    /**
       The source link is either the reference link or additional link.
    */
    Link* sourceLink();
    
    Link* targetLink();

    bool isOverwriting() const;
    bool isOverwritingExistingLink() const;
    bool isAddingLink() const;

    bool updateOverwriting();
    virtual void releaseOverwriteTarget() override;

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
