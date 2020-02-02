/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <cnoid/PlaceableItem>
#include <cnoid/RenderableItem>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class BodyState;
class LinkKinematicsKit;
class InverseKinematics;
class PinDragIK;
class PenetrationBlocker;
class EditableSceneBody;

class CNOID_EXPORT BodyItem : public Item, public PlaceableItem, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyItem();
    BodyItem(const BodyItem& org);
    virtual ~BodyItem();

    Body* body() const;
    void setBody(Body* body);
    virtual void setName(const std::string& name) override;

    bool makeBodyStatic();
    bool makeBodyDynamic();

    //! \deprecated. Use EditableSceneBody::isDraggable().
    bool isEditable() const;
    //! \deprecated. Use EditableSceneBody::setDraggable().
    void setEditable(bool on);

    // API for a composite body
    // The following body and link pair is basically determined by
    // the parent-child relationship in the item tree
    BodyItem* parentBodyItem();
    // True if the body is attached to the parent body with a holder device and an attachment device
    bool isAttachedToParentBody() const;
    // The current parent body can temporarily be changed by this function
    void setTemporalParentBodyItem(BodyItem* parentBodyItem);
    // The parent body item defined by the parent-child relationship in the item tree is restored
    // if the relationship exists. Otherwise, the parent body item is cleared.
    void resetParentBodyItem();
        
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

    // for undo, redo operations
    void beginKinematicStateEdit();
    void cancelKinematicStateEdit();
    void acceptKinematicStateEdit();
    bool undoKinematicState();
    bool redoKinematicState();

    LinkKinematicsKit* getLinkKinematicsKit(Link* targetLink = nullptr, Link* baseLink = nullptr);

    std::shared_ptr<PinDragIK> pinDragIK();
    std::shared_ptr<InverseKinematics> getCurrentIK(Link* targetLink);
    std::shared_ptr<InverseKinematics> getDefaultIK(Link* targetLink);
    std::shared_ptr<PenetrationBlocker> createPenetrationBlocker(Link* link, bool excludeSelfCollisions = false);

    SignalProxy<void()> sigModelUpdated();
    void notifyModelUpdate();
        
    /**
       Signal emitted when there is a change in "kinematic" state such as joint angle of robot,
       joint angular velocity, root position / posture. Item :: sigUpdated () is assumed to be
       a case where the model itself is changed, and it is used distinguished from it.
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
    
    SignalProxy<void()> sigKinematicStateEdited();

    void enableCollisionDetection(bool on);
    bool isCollisionDetectionEnabled() const;
    
    void enableSelfCollisionDetection(bool on);
    bool isSelfCollisionDetectionEnabled() const;        

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

    // PlaceableItem functions
    virtual SignalProxy<void()> sigLocationChanged() override;
    virtual Position getLocation() const override;
    virtual void setLocation(const Position& T) override;
    virtual bool isLocationEditable() const override;

    // RenderableItem function
    virtual SgNode* getScene() override;

    EditableSceneBody* sceneBody();
    EditableSceneBody* existingSceneBody();

    class Impl;

protected:
    virtual Item* doDuplicate() const override;
    virtual void doAssign(Item* item) override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    Impl* impl;
    std::vector<CollisionLinkPairPtr> collisions_;
    std::vector<bool> collisionLinkBitSet_;
    std::vector<std::vector<CollisionLinkPairPtr>> collisionsOfLink_;
    Signal<void()> sigCollisionsUpdated_;

    friend class PyBodyPlugin;
};

typedef ref_ptr<BodyItem> BodyItemPtr;

}

#endif
