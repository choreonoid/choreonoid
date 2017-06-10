/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBody.h"
#include "SceneDevice.h"
#include <cnoid/SceneDrawables>
#include <cnoid/SceneUtil>
#include <cnoid/SceneRenderer>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class LinkShapeTransform : public SgPosTransform
{
    SgNodePtr visualModel;
    SgNodePtr collisionModel;
    bool isVisible;
    
public:
    LinkShapeTransform() : SgPosTransform(findPolymorphicId<LinkShapeTransform>()) { }

    void setVisualModel(SgNode* node) {
        visualModel = node;
        addChild(node);
    }

    void setCollisionModel(SgNode* node) {
        collisionModel = node;
        addChild(node);
    }

    void setVisible(bool on){
        isVisible = on;
    }

    void render(SceneRenderer* renderer){
        renderer->renderCustomTransform(
            this, [=](){ traverse(renderer, renderer->renderingFunctions()); });
    }

    /*
    void preprocess(SceneRenderer* renderer){
        renderer->renderCustomTransform(
            this, [=](){ traverse(renderer, renderer->preprocessFunctions()); });
    }
    */
    
    void traverse(SceneRenderer* renderer, SceneRenderer::NodeFunctionSet* functions){
        int visibility = 0;
        if(isVisible){
            static const SceneRenderer::PropertyKey key("collisionModelVisibility");
            visibility = renderer->property(key, 1);
        }
        for(auto p = cbegin(); p != cend(); ++p){
            SgNode* node = *p;
            if(node == visualModel){
                if(visibility & 1){
                    functions->dispatch(node);
                }
            } else if(node == collisionModel){
                if(visibility & 2){
                    functions->dispatch(node);
                }
            } else {
                functions->dispatch(node);
            }
        }
    }
};

struct NodeTypeRegistration {
    NodeTypeRegistration(){
        SgNode::registerType<LinkShapeTransform, SgPosTransform>();

        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<LinkShapeTransform>(
                    [renderer](SgNode* node){
                        static_cast<LinkShapeTransform*>(node)->render(renderer);
                    });
                /*
                renderer->preprocessFunctions()->setFunction<LinkShapeTransform>(
                    [renderer](SgNode* node){
                        static_cast<CollisionModelVisualizer*>(node)->process(renderer);
                    });
                */
            });
    }
} registration;

}

namespace cnoid {

class SceneLinkImpl
{
public:
    SceneLink* self;
    SgPosTransformPtr shapeTransform;
    SgNodePtr visualShape;
    SgNodePtr collisionShape;
    SgGroup* currentShapeGroup;
    SgGroupPtr shapeGroup;
    bool isVisible;
    bool isVisualShapeVisible;
    bool isCollisionShapeVisible;
    std::vector<SceneDevicePtr> sceneDevices;
    SgGroupPtr deviceGroup;
    float transparency;

    SceneLinkImpl(SceneLink* self, Link* link);
    SceneLinkImpl(SceneLink* self, const SceneLinkImpl& org);
    void setShapeGroup(SgGroup* group);
    int cloneShape(SgCloneMap& cloneMap, bool doNotify);
    int updateVisibility(int action, bool doNotify);
    void setVisibleShapeTypes(bool visual, bool collision);
    void makeTransparent(float transparency, SgCloneMap& cloneMap);
};

class SceneBodyImpl
{
public:
    SceneBody* self;
    SgGroupPtr sceneLinkGroup;
    std::vector<SceneDevicePtr> sceneDevices;
    std::function<SceneLink*(Link*)> sceneLinkFactory;
    bool isVisualShapeVisible;
    bool isCollisionShapeVisible;

    SceneBodyImpl(SceneBody* self, std::function<SceneLink*(Link*)> sceneLinkFactory);
};

}


SceneLink::SceneLink(Link* link)
{
    link_ = link;
    setName(link->name());
    impl = new SceneLinkImpl(this, link);
}


SceneLinkImpl::SceneLinkImpl(SceneLink* self, Link* link)
    : self(self)
{
    shapeTransform = new SgPosTransform;
    shapeTransform->setRotation(link->Rs().transpose());
    self->addChild(shapeTransform);
    visualShape = link->visualShape();
    collisionShape = link->collisionShape();
    currentShapeGroup = shapeTransform;
    isVisible = true;
    isVisualShapeVisible = true;
    isCollisionShapeVisible = false;
    updateVisibility(0, false);
    
    transparency = 0.0;
}


SceneLink::SceneLink(const SceneLink& org)
    : SgPosTransform(org)
{
    link_ = 0;
    impl = new SceneLinkImpl(this, *org.impl);
}


SceneLinkImpl::SceneLinkImpl(SceneLink* self, const SceneLinkImpl& org)
    : self(self)
{
    currentShapeGroup = shapeTransform;
    isVisible = false;
    isVisualShapeVisible = false;
    isCollisionShapeVisible = false;
    transparency = 0.0;
}


SceneLink::~SceneLink()
{
    delete impl;
}


const SgNode* SceneLink::visualShape() const
{
    return impl->visualShape;
}


SgNode* SceneLink::visualShape()
{
    return impl->visualShape;
}


const SgNode* SceneLink::collisionShape() const
{
    return impl->collisionShape;
}


SgNode* SceneLink::collisionShape()
{
    return impl->collisionShape;
}


void SceneLink::setShapeGroup(SgGroup* group)
{
    impl->setShapeGroup(group);
}


void SceneLinkImpl::setShapeGroup(SgGroup* group)
{
    if(group != currentShapeGroup){
        if(shapeGroup){
            shapeTransform->removeChild(shapeGroup);
        }
        if(visualShape){
            currentShapeGroup->removeChild(visualShape);
        }
        if(collisionShape){
            currentShapeGroup->removeChild(collisionShape);
        }
        shapeGroup = group;
        if(shapeGroup){
            currentShapeGroup = shapeGroup;
            shapeTransform->addChild(shapeGroup);
        } else {
            currentShapeGroup = shapeTransform;
        }
        int action = updateVisibility(SgUpdate::ADDED|SgUpdate::REMOVED, false);
        self->notifyUpdate(action);
    }
}


void SceneLink::resetShapeGroup()
{
    impl->setShapeGroup(0);
}


int SceneLinkImpl::cloneShape(SgCloneMap& cloneMap, bool doNotify)
{
    int action = 0;

    SgNode* orgVisualShape = self->link()->visualShape();
    if(orgVisualShape){
        if(currentShapeGroup->removeChild(visualShape)){
            action |= SgUpdate::REMOVED;
        }
        visualShape = orgVisualShape->cloneNode(cloneMap);
    }
    SgNode* orgCollisionShape = self->link()->collisionShape();
    if(orgCollisionShape == orgVisualShape){
        collisionShape = visualShape;
    } else {
        if(orgCollisionShape){
            if(currentShapeGroup->removeChild(collisionShape)){
                action |= SgUpdate::REMOVED;
            }
            collisionShape = orgCollisionShape->cloneNode(cloneMap);
        }
    }
    return updateVisibility(action, doNotify);
}


void SceneLink::cloneShape(SgCloneMap& cloneMap)
{
    impl->cloneShape(cloneMap, true);
}


int SceneLinkImpl::updateVisibility(int action, bool doNotify)
{
    if(visualShape){
        if(isVisible && isVisualShapeVisible){
            if(currentShapeGroup->addChildOnce(visualShape)){
                action |= SgUpdate::ADDED;
            }
        } else {
            if(currentShapeGroup->removeChild(visualShape)){
                action |= SgUpdate::REMOVED;
            }
        }
    }
    if(collisionShape != visualShape){
        if(collisionShape){
            if(isVisible && isCollisionShapeVisible){
                if(currentShapeGroup->addChildOnce(collisionShape)){
                    action |= SgUpdate::ADDED;
                }
            } else {
                if(currentShapeGroup->removeChild(collisionShape)){
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
    if(on != impl->isVisible){
        impl->isVisible = on;
        impl->updateVisibility(0, true);
    }
}


void SceneLink::setVisibleShapeTypes(bool visual, bool collision)
{
    impl->setVisibleShapeTypes(visual, collision);
}


void SceneLinkImpl::setVisibleShapeTypes(bool visual, bool collision)
{
    if(visualShape == collisionShape){
        if(visual || collision){
            visual = true;
        } else {
            visual = false;
        }
        collision = false;
    }
    if(visual != isVisualShapeVisible || collision != isCollisionShapeVisible){
        isVisualShapeVisible = visual;
        isCollisionShapeVisible = collision;
        updateVisibility(0, true);
    }
}


void SceneLink::makeTransparent(float transparency)
{
    SgCloneMap cloneMap;
    cloneMap.setNonNodeCloning(false);
    impl->makeTransparent(transparency, cloneMap);
}


void SceneLinkImpl::makeTransparent(float transparency, SgCloneMap& cloneMap)
{
    if(transparency == this->transparency){
        return;
    }
    this->transparency = transparency;

    if(visualShape){
        int action = 0;
        if(visualShape == self->link()->visualShape()){
            action = cloneShape(cloneMap, false);
        }
        cnoid::makeTransparent(visualShape, transparency, cloneMap, true);
        visualShape->notifyUpdate((SgUpdate::Action)action);
    }
}


void SceneLink::addSceneDevice(SceneDevice* sdev)
{
    if(!impl->deviceGroup){
        impl->deviceGroup = new SgGroup();
        addChild(impl->deviceGroup);
    }
    impl->sceneDevices.push_back(sdev);
    impl->deviceGroup->addChild(sdev);
}


SceneDevice* SceneLink::getSceneDevice(Device* device)
{
    auto& devices = impl->sceneDevices;
    for(size_t i=0; i < devices.size(); ++i){
        SceneDevice* sdev = devices[i];
        if(sdev->device() == device){
            return sdev;
        }
    }
    return 0;
}


SceneBody::SceneBody(Body* body)
    : SceneBody(body, [](Link* link){ return new SceneLink(link); })
{

}


SceneBody::SceneBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory)
    : body_(body)
{
    impl = new SceneBodyImpl(this, sceneLinkFactory);
    addChild(impl->sceneLinkGroup);
    updateModel();
}


SceneBodyImpl::SceneBodyImpl(SceneBody* self, std::function<SceneLink*(Link*)> sceneLinkFactory)
    : self(self),
      sceneLinkFactory(sceneLinkFactory)
{
    sceneLinkGroup = new SgGroup;
    isVisualShapeVisible = true;
    isCollisionShapeVisible = false;
}


SceneBody::SceneBody(const SceneBody& org)
    : SgPosTransform(org)
{

}


SceneBody::~SceneBody()
{
    delete impl;
}


void SceneBody::updateModel()
{
    setName(body_->name());

    if(sceneLinks_.empty()){
        impl->sceneLinkGroup->clearChildren();
        sceneLinks_.clear();
    }
    impl->sceneDevices.clear();
        
    const int n = body_->numLinks();
    for(int i=0; i < n; ++i){
        Link* link = body_->link(i);
        SceneLink* sLink = impl->sceneLinkFactory(link);
        impl->sceneLinkGroup->addChild(sLink);
        sceneLinks_.push_back(sLink);
    }

    const DeviceList<Device>& devices = body_->devices();
    for(size_t i=0; i < devices.size(); ++i){
        Device* device = devices[i];
        SceneDevice* sceneDevice = SceneDevice::create(device);
        if(sceneDevice){
            sceneLinks_[device->link()->index()]->addSceneDevice(sceneDevice);
            impl->sceneDevices.push_back(sceneDevice);
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
    if(visual != impl->isVisualShapeVisible || collision != impl->isCollisionShapeVisible){
        for(size_t i=0; i < sceneLinks_.size(); ++i){
            sceneLinks_[i]->setVisibleShapeTypes(visual, collision);
        }
        impl->isVisualShapeVisible = visual;
        impl->isCollisionShapeVisible = collision;
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
    auto& sceneDevices = impl->sceneDevices;
    for(size_t i=0; i < sceneDevices.size(); ++i){
        sceneDevices[i]->setSceneUpdateConnection(on);
    }
}


void SceneBody::updateSceneDevices(double time)
{
    auto& sceneDevices = impl->sceneDevices;
    for(size_t i=0; i < sceneDevices.size(); ++i){
        sceneDevices[i]->updateScene(time);
    }
}


void SceneBody::makeTransparent(float transparency, SgCloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->impl->makeTransparent(transparency, cloneMap);
    }
}


void SceneBody::makeTransparent(float transparency)
{
    SgCloneMap cloneMap;
    cloneMap.setNonNodeCloning(false);
    makeTransparent(transparency, cloneMap);
}
