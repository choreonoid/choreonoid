/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBody.h"
#include "SceneDevice.h"
#include <cnoid/SceneShape>
#include <cnoid/SceneUtil>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


SceneLink::SceneLink(Link* link)
    : link_(link)
{
    setName(link->name());

    visualShape_ = link->visualShape();
    collisionShape_ = link->collisionShape();
    currentShapeGroup = this;
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
    currentShapeGroup = this;
    isVisible_ = false;
    isVisualShapeVisible_ = false;
    isCollisionShapeVisible_ = false;
    transparency_ = 0.0;
}


void SceneLink::setShapeGroup(SgGroup* group)
{
    if(group != currentShapeGroup){
        if(shapeGroup){
            removeChild(shapeGroup);
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
            addChild(shapeGroup);
        } else {
            currentShapeGroup = this;
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


namespace {
SceneLink* createSceneLink(Link* link)
{
    return new SceneLink(link);
}
}


SceneBody::SceneBody(BodyPtr body)
{
    initialize(body, createSceneLink);
}


SceneBody::SceneBody(BodyPtr body, boost::function<SceneLink*(Link*)> sceneLinkFactory)
{
    initialize(body, sceneLinkFactory);
}


void SceneBody::initialize(BodyPtr& body, const boost::function<SceneLink*(Link*)>& sceneLinkFactory)
{
    this->sceneLinkFactory = sceneLinkFactory;
    body_ = body;
    sceneLinkGroup = new SgGroup;
    addChild(sceneLinkGroup);
    updateModel();
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
    updateSceneDevices();
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
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->setVisibleShapeTypes(visual, collision);
    }
}


void SceneBody::updateLinkPositions()
{
    const int n = sceneLinks_.size();
    for(int i=0; i < n; ++i){
        SceneLinkPtr& sLink = sceneLinks_[i];
        sLink->setPosition(sLink->link()->position());
    }
}


void SceneBody::updateLinkPositions(SgUpdate& update)
{
    const int n = sceneLinks_.size();
    for(int i=0; i < n; ++i){
        SceneLinkPtr& sLink = sceneLinks_[i];
        sLink->setPosition(sLink->link()->position());
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


void SceneBody::updateSceneDevices()
{
    for(size_t i=0; i < sceneDevices.size(); ++i){
        sceneDevices[i]->updateScene();
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
