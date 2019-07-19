/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_ITEM_H

#include <cnoid/Item>
#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <cnoid/SceneProvider>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

class BodyState;
class BodyItem;
typedef ref_ptr<BodyItem> BodyItemPtr;
class BodyItemImpl;
class InverseKinematics;
class PinDragIK;
class PenetrationBlocker;
class EditableSceneBody;

class CNOID_EXPORT BodyItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyItem();
    BodyItem(const BodyItem& org);
    virtual ~BodyItem();

    bool loadModelFile(const std::string& filename);

    void setBody(Body* body);
            
    virtual void setName(const std::string& name) override;

    Body* body() const;

    bool isEditable() const;
    void setEditable(bool on);
        
    enum PresetPoseID { INITIAL_POSE, STANDARD_POSE };

    void moveToOrigin();

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
            
    virtual SgNode* getScene() override;
    EditableSceneBody* sceneBody();
    EditableSceneBody* existingSceneBody();

protected:
    virtual Item* doDuplicate() const override;
    virtual void doAssign(Item* item) override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    friend class BodyItemImpl;
    friend class PyBodyPlugin;
    BodyItemImpl* impl;
    std::vector<CollisionLinkPairPtr> collisions_;
    std::vector<bool> collisionLinkBitSet_;
    std::vector<std::vector<CollisionLinkPairPtr>> collisionsOfLink_;
    Signal<void()> sigCollisionsUpdated_;
};

}

#endif
