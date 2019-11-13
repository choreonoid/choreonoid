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

class SceneLinkImpl;

class CNOID_EXPORT SceneLink : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    SceneLink(Link* link);
    ~SceneLink();

    Link* link() { return link_; }
    const Link* link() const { return link_; }

    const SgNode* visualShape() const;
    SgNode* visualShape();
    const SgNode* collisionShape() const;
    SgNode* collisionShape();

    void insertEffectGroup(SgGroup* group);
    void removeEffectGroup(SgGroup* group);
    
    void setVisible(bool on);
    void makeTransparent(float transparency);
    
    void addSceneDevice(SceneDevice* sdev);
    SceneDevice* getSceneDevice(Device* device);

private:
    SceneLink(const SceneLink& org);

    Link* link_;
    SceneLinkImpl* impl;
    friend class SceneBody;
};

typedef ref_ptr<SceneLink> SceneLinkPtr;

class SceneBodyImpl;
    
class CNOID_EXPORT SceneBody : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    SceneBody(Body* body);
    SceneBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory);
    virtual ~SceneBody();

    Body* body() { return body_; }
    const Body* body() const { return body_; }

    void cloneShapes(CloneMap& cloneMap);

    int numSceneLinks() const { return sceneLinks_.size(); }
    SceneLink* sceneLink(int index) { return sceneLinks_[index]; }
    const SceneLink* sceneLink(int index) const { return sceneLinks_[index]; }

    void updateLinkPositions();
    void updateLinkPositions(SgUpdate& update);

    SceneDevice* getSceneDevice(Device* device);
    void setSceneDeviceUpdateConnection(bool on);
    void updateSceneDevices(double time);

    void makeTransparent(float transparency);
    void makeTransparent(float transparency, CloneMap& cloneMap);

    virtual void updateModel();

private:
    SceneBody(const SceneBody& org);

    BodyPtr body_;
    std::vector<SceneLinkPtr> sceneLinks_;
    SceneBodyImpl* impl;
};
            
typedef ref_ptr<SceneBody> SceneBodyPtr;

}
    
#endif
