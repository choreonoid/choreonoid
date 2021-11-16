/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <cnoid/LocatableItem>
#include <cnoid/RenderableItem>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class ItemManager;
class BodyState;
class LinkKinematicsKit;
class InverseKinematics;
class PinDragIK;
class PenetrationBlocker;
class EditableSceneBody;
class ItemFileIO;

class CNOID_EXPORT BodyItem : public Item, public LocatableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    // The following functions are Implemented in BodyItemFileIO.cpp
    static void registerBodyItemFileIoSet(ItemManager* im);
    //! The actual type of the IO object returned by this function is BodyItemBodyFileIO.
    static ItemFileIO* bodyFileIO();
    static ItemFileIO* meshFileIO();
        
    BodyItem();
    BodyItem(const std::string& name);
    BodyItem(const BodyItem& org);
    virtual ~BodyItem();

    virtual bool setName(const std::string& name) override;

    Body* body() const;
    void setBody(Body* body);

    bool isSharingShapes() const;
    void cloneShapes(CloneMap& cloneMap);

    bool makeBodyStatic();
    bool makeBodyDynamic();

    // API for a composite body
    // The following body and link pair is basically determined by
    // the parent-child relationship in the item tree
    BodyItem* parentBodyItem();
    // True if the body is attached to the parent body with a holder device and an attachment device
    bool isAttachedToParentBody() const { return isAttachedToParentBody_; }
    void setAttachmentEnabled(bool on, bool doNotifyUpdate = true);
    bool isAttachmentEnabled() const;
    bool attachToParentBody(bool doNotifyUpdate = true);    

    // The current parent body can temporarily be changed by this function
    //void setTemporalParentBodyItem(BodyItem* parentBodyItem);
    // The parent body item defined by the parent-child relationship in the item tree is restored
    // if the relationship exists. Otherwise, the parent body item is cleared.
    //void resetParentBodyItem();
        
    void moveToOrigin();
    enum PresetPoseID { INITIAL_POSE, STANDARD_POSE };
    void setPresetPose(PresetPoseID id);

    Link* currentBaseLink() const;
    void setCurrentBaseLink(Link* link);

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false);

    void copyKinematicState();
    void pasteKinematicState();

    void storeKinematicState(BodyState& state);
    bool restoreKinematicState(const BodyState& state);

    void storeInitialState();
    void restoreInitialState(bool doNotify = true);

    void getInitialState(BodyState& out_state);
    void setInitialState(const BodyState& in_state);

    [[deprecated("This function does nothing.")]]
    void beginKinematicStateEdit();
    [[deprecated("This function does nothing.")]]
    void cancelKinematicStateEdit();
    [[deprecated("Use notifyKinematicStateEdited")]]
    void acceptKinematicStateEdit();

    LinkKinematicsKit* findPresetLinkKinematicsKit(Link* targetLink = nullptr);
    std::shared_ptr<InverseKinematics> findPresetIK(Link* targetLink);
    LinkKinematicsKit* getCurrentLinkKinematicsKit(Link* targetLink);
    std::shared_ptr<InverseKinematics> getCurrentIK(Link* targetLink);
    std::shared_ptr<PinDragIK> getOrCreatePinDragIK();
    std::shared_ptr<PinDragIK> checkPinDragIK();
    std::shared_ptr<PenetrationBlocker> createPenetrationBlocker(Link* link, bool excludeSelfCollisions = false);

    [[deprecated("Create a new BodyItem and use Item::replace to update the model.")]]
    SignalProxy<void()> sigModelUpdated();
    [[deprecated("Create a new BodyItem and use Item::replace to update the model.")]]
    void notifyModelUpdate();
        
    /**
       This signal is emitted when there is a change in "kinematic" state such as joint angle of robot,
       joint angular velocity, root position / posture. Item :: sigUpdated () is assumed to be
       a case where the model itself is changed, and you have to distinguish them.
    */
    SignalProxy<void()> sigKinematicStateChanged();

    void notifyKinematicStateChange(
        bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);
    void notifyKinematicStateChange(
        Connection& connectionToBlock,
        bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);
    void notifyKinematicStateChangeLater(
        bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);
    void notifyKinematicStateChangeLater(
        Connection& connectionToBlock,
        bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);

    /**
       This signal is emitted when a kinematic state has been updated.
       In constrast to sigKinematicStateChange, this signal is emitted when a series of changes
       are finalized.
    */
    SignalProxy<void()> sigKinematicStateUpdated();

    void notifyKinematicStateUpdate(bool doNotifyStateChange = true);
    
    bool isCollisionDetectionEnabled() const;
    void setCollisionDetectionEnabled(bool on);
    [[deprecated("Use setCollisionDetectionEnabled")]]
    void enableCollisionDetection(bool on) { setCollisionDetectionEnabled(on); }
    
    bool isSelfCollisionDetectionEnabled() const;        
    void setSelfCollisionDetectionEnabled(bool on);
    [[deprecated("Use setSelfCollisionDetectionEnabled")]]
    void enableSelfCollisionDetection(bool on) { setSelfCollisionDetectionEnabled(on); }
    
    void clearCollisions();

    std::vector<CollisionLinkPairPtr>& collisions() { return collisions_; }
    const std::vector<CollisionLinkPairPtr>& collisions() const { return collisions_; }
    std::vector<bool>& collisionLinkBitSet() { return collisionLinkBitSet_; }
    const std::vector<bool>& collisionLinkBitSet() const { return collisionLinkBitSet_; }
    std::vector<CollisionLinkPairPtr>& collisionsOfLink(int linkIndex) { return collisionsOfLink_[linkIndex]; }
    const std::vector<CollisionLinkPairPtr>& collisionsOfLink(int linkIndex) const { return collisionsOfLink_[linkIndex]; }
    SignalProxy<void()> sigCollisionsUpdated() { return sigCollisionsUpdated_; }
    void notifyCollisionUpdate() { sigCollisionsUpdated_(); }

    const Vector3& centerOfMass();

    bool isLeggedBody() const;
    bool doLegIkToMoveCm(const Vector3& c, bool onlyProjectionToFloor = false);

    const Vector3& zmp() const;
    void setZmp(const Vector3& zmp);
    void editZmp(const Vector3& zmp);

    enum PositionType { CM_PROJECTION, HOME_COP, RIGHT_HOME_COP, LEFT_HOME_COP, ZERO_MOMENT_POINT };
            
    stdx::optional<Vector3> getParticularPosition(PositionType posType);

    bool setStance(double width);

    // LocatableItem function
    virtual LocationProxyPtr getLocationProxy() override;
    
    bool isLocationEditable() const;
    void setLocationEditable(bool on);
    LocationProxyPtr createLinkLocationProxy(Link* link);

    // RenderableItem function
    virtual SgNode* getScene() override;

    EditableSceneBody* sceneBody();
    EditableSceneBody* existingSceneBody();
    float transparency() const;
    void setTransparency(float t);

    bool isVisibleLinkSelectionMode() const { return isVisibleLinkSelectionMode_; }
    void setVisibleLinkSelectionMode(bool on) { isVisibleLinkSelectionMode_ = on; }

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual bool doAssign(const Item* item) override;
    virtual void onTreePathChanged() override;
    virtual void onConnectedToRoot() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
            
private:
    Impl* impl;
    bool isAttachedToParentBody_;
    bool isVisibleLinkSelectionMode_;
    std::vector<CollisionLinkPairPtr> collisions_;
    std::vector<bool> collisionLinkBitSet_;
    std::vector<std::vector<CollisionLinkPairPtr>> collisionsOfLink_;
    Signal<void()> sigCollisionsUpdated_;

    friend class PyBodyPlugin;
};

typedef ref_ptr<BodyItem> BodyItemPtr;

}

#endif
