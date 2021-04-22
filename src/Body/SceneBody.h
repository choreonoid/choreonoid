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

class SceneBody;
class SceneLinkImpl;

class CNOID_EXPORT SceneLink : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    SceneLink(SceneBody* sceneBody, Link* link);
    SceneLink(const SceneLink& org) = delete;
    ~SceneLink();

    SceneBody* sceneBody(){ return sceneBody_; }
    const SceneBody* sceneBody() const { return sceneBody_; }

    Link* link() { return link_; }
    const Link* link() const { return link_; }

    const SgNode* visualShape() const;
    SgNode* visualShape();
    const SgNode* collisionShape() const;
    SgNode* collisionShape();

    void insertEffectGroup(SgGroup* effect, SgUpdateRef update = SgUpdateRef());
    void removeEffectGroup(SgGroup* effect, SgUpdateRef update = SgUpdateRef());
    
    void setVisible(bool on);
    float transparency() const;
    void setTransparency(float transparency, SgUpdateRef update = SgUpdateRef());
    //! \deprecated. Use setTransparency.
    void makeTransparent(float transparency);
    
    void addSceneDevice(SceneDevice* sdev);
    SceneDevice* getSceneDevice(Device* device);

private:
    Link* link_;
    SceneLinkImpl* impl;
    SceneBody* sceneBody_;
    friend class SceneBody;
};

typedef ref_ptr<SceneLink> SceneLinkPtr;

class SceneBodyImpl;
    
class CNOID_EXPORT SceneBody : public SgPosTransform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneBody();
    SceneBody(Body* body);
    SceneBody(const SceneBody& org) = delete;
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

    void setTransparency(float transparency);

    //! \deprecated. Use setTransparency.
    void makeTransparent(float transparency);
    //! \deprecated. Use setTransparency.
    void makeTransparent(float transparency, CloneMap& cloneMap);

    void insertEffectGroup(SgGroup* effect, SgUpdateRef update = SgUpdateRef());
    void removeEffectGroup(SgGroup* effect, SgUpdateRef update = SgUpdateRef());

    virtual void updateModel();

protected:
    void setBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory);

private:
    BodyPtr body_;
    std::vector<SceneLinkPtr> sceneLinks_;
    SceneBodyImpl* impl;

    friend class SceneLink;
};
            
typedef ref_ptr<SceneBody> SceneBodyPtr;

}
    
#endif
