/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_SCENE_BODY_H
#define CNOID_BODY_SCENE_BODY_H

#include "Body.h"
#include "SceneDevice.h"
#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneLink : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    SceneLink(Link* link);

    Link* link() { return link_; }
    const Link* link() const { return link_; }

    const SgNode* visualShape() const { return visualShape_; }
    SgNode* visualShape() { return visualShape_; }
    const SgNode* collisionShape() const { return collisionShape_; }
    SgNode* collisionShape() { return collisionShape_; }
    void setShapeGroup(SgGroup* group);
    void resetShapeGroup();
    void cloneShape(SgCloneMap& cloneMap);
    void setVisible(bool on);
    void setVisibleShapeTypes(bool visual, bool collision);
    void makeTransparent(float transparency);
    void makeTransparent(float transparency, SgCloneMap& cloneMap);
    
    void addSceneDevice(SceneDevice* sdev);
    SceneDevice* getSceneDevice(Device* device);

private:
    SceneLink(const SceneLink& org);
    Link* link_;
    SgNodePtr visualShape_;
    SgNodePtr collisionShape_;
    SgGroup* currentShapeGroup;
    SgGroupPtr shapeGroup;
    bool isVisible_;
    bool isVisualShapeVisible_;
    bool isCollisionShapeVisible_;
    std::vector<SceneDevicePtr> sceneDevices_;
    SgGroupPtr deviceGroup;
    float transparency_;

    int cloneShape(SgCloneMap& cloneMap, bool doNotify);
    int updateVisibility(int action, bool doNotify);
};
typedef ref_ptr<SceneLink> SceneLinkPtr;
    
    
class CNOID_EXPORT SceneBody : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
    SceneBody(BodyPtr body);
    SceneBody(BodyPtr body, boost::function<SceneLink*(Link*)> sceneLinkFactory);

    Body* body() { return body_; }
    const Body* body() const { return body_; }

    void cloneShapes(SgCloneMap& cloneMap);

    void setVisibleShapeTypes(bool visual, bool collision);

    int numSceneLinks() const { return sceneLinks_.size(); }
    SceneLink* sceneLink(int index) { return sceneLinks_[index]; }
    const SceneLink* sceneLink(int index) const { return sceneLinks_[index]; }

    void updateLinkPositions();
    void updateLinkPositions(SgUpdate& update);

    SceneDevice* getSceneDevice(Device* device);
    void setSceneDeviceUpdateConnection(bool on);
    void updateSceneDevices();

    void makeTransparent(float transparency);
    void makeTransparent(float transparency, SgCloneMap& cloneMap);

    virtual void updateModel();

protected:
    BodyPtr body_;
    SgGroupPtr sceneLinkGroup;
    std::vector<SceneLinkPtr> sceneLinks_;
    std::vector<SceneDevicePtr> sceneDevices;

    virtual ~SceneBody();

private:
    boost::function<SceneLink*(Link*)> sceneLinkFactory;

    SceneBody(const SceneBody& org);
    void initialize(BodyPtr& body, const boost::function<SceneLink*(Link*)>& sceneLinkFactory);
};
            
typedef ref_ptr<SceneBody> SceneBodyPtr;
}
    
#endif
