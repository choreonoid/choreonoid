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
#include <boost/dynamic_bitset.hpp>
#include <boost/optional.hpp>
#include "exportdecl.h"

namespace cnoid {

class BodyState;
class BodyItem;
typedef ref_ptr<BodyItem> BodyItemPtr;
class BodyItemImpl;
class InverseKinematics;
typedef boost::shared_ptr<InverseKinematics> InverseKinematicsPtr;
class PinDragIK;
typedef boost::shared_ptr<PinDragIK> PinDragIKptr;
class PenetrationBlocker;
typedef boost::shared_ptr<PenetrationBlocker> PenetrationBlockerPtr;
class EditableSceneBody;

class CNOID_EXPORT BodyItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyItem();
    BodyItem(const BodyItem& org);
    virtual ~BodyItem();

    bool loadModelFile(const std::string& filename);
            
    virtual void setName(const std::string& name);

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
    void restoreInitialState();

    void getInitialState(BodyState& out_state);
    void setInitialState(const BodyState& in_state);

    // for undo, redo operations
    void beginKinematicStateEdit();
    void acceptKinematicStateEdit();
    bool undoKinematicState();
    bool redoKinematicState();

    PinDragIKptr pinDragIK();
    InverseKinematicsPtr getCurrentIK(Link* targetLink);
    InverseKinematicsPtr getDefaultIK(Link* targetLink);
    PenetrationBlockerPtr createPenetrationBlocker(Link* link, bool excludeSelfCollisions = false);

    SignalProxy<void()> sigModelUpdated();
    void notifyModelUpdate();
        
    /**
       @if jp
       ロボットの関節角、関節角速度、root位置・姿勢などの「運動学的」状態に変更が生じたときに
       発行されるシグナル。
       Item::sigUpdated() はモデル自体が変わった場合とし、そちらとは区別して使う。
       @endif
    */
    SignalProxy<void()> sigKinematicStateChanged();

    void notifyKinematicStateChange(
        bool requestFK = false, bool requestVelFK = false, bool requestAccFK = false);
            
    void notifyKinematicStateChange(
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
    boost::dynamic_bitset<>& collisionLinkBitSet() { return collisionLinkBitSet_; }
    const boost::dynamic_bitset<>& collisionLinkBitSet() const { return collisionLinkBitSet_; }
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
            
    boost::optional<Vector3> getParticularPosition(PositionType posType);

    bool setStance(double width);
            
    virtual SgNode* getScene();
    EditableSceneBody* sceneBody();
    EditableSceneBody* existingSceneBody();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doAssign(Item* item);
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    friend class BodyItemImpl;
    friend class PyBodyPlugin;
    BodyItemImpl* impl;
    std::vector<CollisionLinkPairPtr> collisions_;
    boost::dynamic_bitset<> collisionLinkBitSet_;
    std::vector< std::vector<CollisionLinkPairPtr> > collisionsOfLink_;
    Signal<void()> sigCollisionsUpdated_;
};

}

#endif
