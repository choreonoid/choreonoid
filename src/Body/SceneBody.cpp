/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBody.h"
#include "SceneDevice.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid

namespace {

class VisualGroup : public SgGroup
{
public:
    VisualGroup() : SgGroup(findPolymorphicId<VisualGroup>()) { }
    
    void render(SceneRenderer* renderer){
        static const SceneRenderer::PropertyKey key("isVisualModelVisible");
        if(renderer->property(key, false)){
            renderer->renderingFunctions().dispatchAs<SgGroup>(this);
        }
    }
};

class CollisionGroup : public SgGroup
{
public:
    CollisionGroup() : SgGroup(findPolymorphicId<CollisionGroup>()) { }
    
    void render(SceneRenderer* renderer){
        static const SceneRenderer::PropertyKey key("isCollisionModelVisible");
        if(renderer->property(key, false)){
            renderer->renderingFunctions().dispatchAs<SgGroup>(this);
        }
    }
};

struct NodeTypeRegistration {
    NodeTypeRegistration(){
        SgNode::registerType<CollisionShapeSwitch, SgNode>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                auto& functions = renderer->renderingFunctions();
                functions.setFunction<CollisionShapeSwitch>(
                    [renderer](SgNode* node){
                        static_cast<CollisionShapeSwitch*>(node)->render(renderer);
                    });
            });
    }
} registration;

}


SceneLink::SceneLink(Link* link)
    : link_(link)
{
    setName(link->name());

    shapeTransform = new SgPosTransform;
    shapeTransform->setRotation(link->Rs().transpose());
    addChild(shapeTransform);
    visualShape_ = link->visualShape();
    collisionShape_ = link->collisionShape();
    currentShapeGroup = shapeTransform;
    isVisible_ = true;
    isVisualShapeVisible_ = true;
    isCollisionShapeVisible_ = false;
    updateVisibility(0, false);
    
    transparency_ = 0.0;
}


SceneLink::SceneLink(const SceneLink& org)
    : SgPosTransform(org)
{
    link_ = 0;
    currentShapeGroup = shapeTransform;
    isVisible_ = false;
    isVisualShapeVisible_ = false;
    isCollisionShapeVisible_ = false;
    transparency_ = 0.0;
}


void SceneLink::setShapeGroup(SgGroup* group)
{
    if(group != currentShapeGroup){
        if(shapeGroup){
            shapeTransform->removeChild(shapeGroup);
        }
        if(visualShape_){
            currentShapeGroup->removeChild(visualShape_);
        }
        if(collisionShape_){
            currentShapeGroup->removeChild(collisionShape_);
        }
        shapeGroup = group;
        if(shapeGroup){
            currentShapeGroup = shapeGroup;
            shapeTransform->addChild(shapeGroup);
        } else {
            currentShapeGroup = shapeTransform;
        }
        int action = updateVisibility(SgUpdate::ADDED|SgUpdate::REMOVED, false);
        notifyUpdate(action);
    }
}


void SceneLink::resetShapeGroup()
{
    setShapeGroup(0);
}


int SceneLink::cloneShape(SgCloneMap& cloneMap, bool doNotify)
{
    int action = 0;

    SgNode* orgVisualShape = link_->visualShape();
    if(orgVisualShape){
        if(currentShapeGroup->removeChild(visualShape_)){
            action |= SgUpdate::REMOVED;
        }
        visualShape_ = orgVisualShape->cloneNode(cloneMap);
    }
    SgNode* orgCollisionShape = link_->collisionShape();
    if(orgCollisionShape == orgVisualShape){
        collisionShape_ = visualShape_;
    } else {
        if(orgCollisionShape){
            if(currentShapeGroup->removeChild(collisionShape_)){
                action |= SgUpdate::REMOVED;
            }
            collisionShape_ = orgCollisionShape->cloneNode(cloneMap);
        }
    }
    return updateVisibility(action, doNotify);
}


void SceneLink::cloneShape(SgCloneMap& cloneMap)
{
    cloneShape(cloneMap, true);
}


int SceneLink::updateVisibility(int action, bool doNotify)
{
    if(visualShape_){
        if(isVisible_ && isVisualShapeVisible_){
            if(currentShapeGroup->addChildOnce(visualShape_)){
                action |= SgUpdate::ADDED;
            }
        } else {
            if(currentShapeGroup->removeChild(visualShape_)){
                action |= SgUpdate::REMOVED;
            }
        }
    }
    if(collisionShape_ != visualShape_){
        if(collisionShape_){
            if(isVisible_ && isCollisionShapeVisible_){
                if(currentShapeGroup->addChildOnce(collisionShape_)){
                    action |= SgUpdate::ADDED;
                }
            } else {
                if(currentShapeGroup->removeChild(collisionShape_)){
                    action |= SgUpdate::REMOVED;
                }
            }
        }
    }
    if(action && doNotify){
        currentShapeGroup->notifyUpdate((SgUpdate::Action)action);
    }
    return action;
}


void SceneLink::setVisible(bool on)
{
    if(on != isVisible_){
        isVisible_ = on;
        updateVisibility(0, true);
    }
}


void SceneLink::setVisibleShapeTypes(bool visual, bool collision)
{
    if(visualShape_ == collisionShape_){
        if(visual || collision){
            visual = true;
        } else {
            visual = false;
        }
        collision = false;
    }
    if(visual != isVisualShapeVisible_ || collision != isCollisionShapeVisible_){
        isVisualShapeVisible_ = visual;
        isCollisionShapeVisible_ = collision;
        updateVisibility(0, true);
    }
}


void SceneLink::makeTransparent(float transparency, SgCloneMap& cloneMap)
{
    if(transparency == transparency_){
        return;
    }
    transparency_ = transparency;

    if(visualShape_){
        int action = 0;
        if(visualShape_ == link_->visualShape()){
            action = cloneShape(cloneMap, false);
        }
        cnoid::makeTransparent(visualShape_, transparency, cloneMap, true);
        visualShape_->notifyUpdate((SgUpdate::Action)action);
    }
}


void SceneLink::makeTransparent(float transparency)
{
    SgCloneMap cloneMap;
    cloneMap.setNonNodeCloning(false);
    makeTransparent(transparency, cloneMap);
}


void SceneLink::addSceneDevice(SceneDevice* sdev)
{
    if(!deviceGroup){
        deviceGroup = new SgGroup();
        addChild(deviceGroup);
    }
    sceneDevices_.push_back(sdev);
    deviceGroup->addChild(sdev);
}


SceneDevice* SceneLink::getSceneDevice(Device* device)
{
    for(size_t i=0; i < sceneDevices_.size(); ++i){
        SceneDevice* sdev = sceneDevices_[i];
        if(sdev->device() == device){
            return sdev;
        }
    }
    return 0;
}


static SceneLink* createSceneLink(Link* link)
{
    return new SceneLink(link);
}


SceneBody::SceneBody(Body* body)
{
    initialize(body, createSceneLink);
}


SceneBody::SceneBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory)
{
    initialize(body, sceneLinkFactory);
}


void SceneBody::initialize(Body* body, const std::function<SceneLink*(Link*)>& sceneLinkFactory)
{
    this->sceneLinkFactory = sceneLinkFactory;
    body_ = body;
    sceneLinkGroup = new SgGroup;
    addChild(sceneLinkGroup);
    updateModel();
    isVisualShapeVisible = true;
    isCollisionShapeVisible = false;
}


SceneBody::SceneBody(const SceneBody& org)
    : SgPosTransform(org)
{

}


SceneBody::~SceneBody()
{

}


void SceneBody::updateModel()
{
    setName(body_->name());

    if(sceneLinks_.empty()){
        sceneLinkGroup->clearChildren();
        sceneLinks_.clear();
    }
    sceneDevices.clear();
        
    const int n = body_->numLinks();
    for(int i=0; i < n; ++i){
        Link* link = body_->link(i);
        SceneLink* sLink = sceneLinkFactory(link);
        sceneLinkGroup->addChild(sLink);
        sceneLinks_.push_back(sLink);
    }

    const DeviceList<Device>& devices = body_->devices();
    for(size_t i=0; i < devices.size(); ++i){
        Device* device = devices[i];
        SceneDevice* sceneDevice = SceneDevice::create(device);
        if(sceneDevice){
            sceneLinks_[device->link()->index()]->addSceneDevice(sceneDevice);
            sceneDevices.push_back(sceneDevice);
        }
    }

    updateLinkPositions();
    updateSceneDevices(0.0);
    notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED);
}


void SceneBody::cloneShapes(SgCloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->cloneShape(cloneMap);
    }
}


void SceneBody::setVisibleShapeTypes(bool visual, bool collision)
{
    if(visual != isVisualShapeVisible || collision != isCollisionShapeVisible){
        for(size_t i=0; i < sceneLinks_.size(); ++i){
            sceneLinks_[i]->setVisibleShapeTypes(visual, collision);
        }
        isVisualShapeVisible = visual;
        isCollisionShapeVisible = collision;
    }
}


void SceneBody::updateLinkPositions()
{
    const int n = sceneLinks_.size();
    for(int i=0; i < n; ++i){
        SceneLinkPtr& sLink = sceneLinks_[i];
        sLink->setRotation(sLink->link()->attitude());
        sLink->setTranslation(sLink->link()->translation());
    }
}


void SceneBody::updateLinkPositions(SgUpdate& update)
{
    const int n = sceneLinks_.size();
    for(int i=0; i < n; ++i){
        SceneLinkPtr& sLink = sceneLinks_[i];
        sLink->setRotation(sLink->link()->attitude());
        sLink->setTranslation(sLink->link()->translation());
        sLink->notifyUpdate(update);
    }
}


SceneDevice* SceneBody::getSceneDevice(Device* device)
{
    const int linkIndex = device->link()->index();
    if(linkIndex < sceneLinks_.size()){
        return sceneLinks_[linkIndex]->getSceneDevice(device);
    }
    return 0;
}


void SceneBody::setSceneDeviceUpdateConnection(bool on)
{
    for(size_t i=0; i < sceneDevices.size(); ++i){
        sceneDevices[i]->setSceneUpdateConnection(on);
    }
}


void SceneBody::updateSceneDevices(double time)
{
    for(size_t i=0; i < sceneDevices.size(); ++i){
        sceneDevices[i]->updateScene(time);
    }
}


void SceneBody::makeTransparent(float transparency, SgCloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->makeTransparent(transparency, cloneMap);
    }
}


void SceneBody::makeTransparent(float transparency)
{
    SgCloneMap cloneMap;
    cloneMap.setNonNodeCloning(false);
    makeTransparent(transparency, cloneMap);
}
