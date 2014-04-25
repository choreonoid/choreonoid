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
    orgShape_ = link->shape();
    if(orgShape_){
        shape_ = orgShape_;
        addChild(shape_);
    }
    transparency_ = 0.0;
}


SceneLink::SceneLink(const SceneLink& org)
    : SgPosTransform(org)
{
    link_ = 0;
    transparency_ = 0.0;
}


void SceneLink::cloneShape(SgCloneMap& cloneMap)
{
    if(orgShape_){
        removeChild(shape_);
        shape_ = orgShape_->cloneNode(cloneMap);
        addChild(shape_);
        notifyUpdate((SgUpdate::Action)(SgUpdate::ADDED | SgUpdate::REMOVED));
    }
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
        SceneDevice* sdev = sceneDevices_[i].get();
        if(sdev->device() == device){
            return sdev;
        }
    }
    return 0;
}


bool SceneLink::isVisible() const
{
    return (shape_ && contains(shape_));
}

        
void SceneLink::setVisible(bool on)
{
    if(shape_){
        if(!contains(shape_) && on){
            addChild(shape_);
        } else if(contains(shape_) && !on){
            removeChild(shape_);
        }
    }
}


void SceneLink::makeTransparent(float transparency, SgCloneMap& cloneMap)
{
    if(transparency == transparency_){
        return;
    }
    transparency_ = transparency;
        
    if(orgShape_){
        removeChild(shape_);
        if(transparency == 0.0){
            shape_ = orgShape_;
        } else {
            shape_ = orgShape_->cloneNode(cloneMap);
            cnoid::makeTransparent(shape_, transparency, cloneMap, true);
        }
        addChild(shape_);
        notifyUpdate((SgUpdate::Action)(SgUpdate::ADDED | SgUpdate::REMOVED));
    }
}


void SceneLink::makeTransparent(float transparency)
{
    SgCloneMap cloneMap;
    cloneMap.setNonNodeCloning(false);
    makeTransparent(transparency, cloneMap);
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
    body_ = body;
    setName(body->name());
    
    const int n = body->numLinks();
    for(int i=0; i < n; ++i){
        Link* link = body->link(i);
        SceneLink* sLink = sceneLinkFactory(link);
        addChild(sLink);
        sceneLinks_.push_back(sLink);
    }

    const DeviceList<Device>& devices = body->devices();
    for(size_t i=0; i < devices.size(); ++i){
        Device* device = devices.get(i);
        SceneDevice* sceneDevice = new SceneDevice(device);
        sceneLinks_[device->link()->index()]->addSceneDevice(sceneDevice);
        sceneDevices.push_back(sceneDevice);
    }
}


SceneBody::SceneBody(const SceneBody& org)
    : SgPosTransform(org)
{

}


SceneBody::~SceneBody()
{

}


void SceneBody::cloneShapes(SgCloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->cloneShape(cloneMap);
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
