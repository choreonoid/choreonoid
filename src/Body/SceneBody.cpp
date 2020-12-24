/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBody.h"
#include <cnoid/SceneNodeClassRegistry>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneEffects>
#include <cnoid/SceneUtil>
#include <cnoid/SceneRenderer>
#include <cnoid/CloneMap>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class LinkShapeGroup : public SgGroup
{
public:
    SgNodePtr visualShape;
    SgNodePtr collisionShape;
    ScopedConnection collisionShapeUpdateConnection;
    bool isVisible;
    bool hasClone;
    
    LinkShapeGroup(Link* link)
        : SgGroup(findClassId<LinkShapeGroup>())
    {
        visualShape = link->visualShape();
        if(visualShape){
            addChild(visualShape);
        }
        collisionShape = link->collisionShape();
        resetCollisionShapeUpdateConnection();

        isVisible = true;
        hasClone = false;
    }

    void setVisible(bool on)
    {
        isVisible = on;
    }

    void cloneShapes(CloneMap& cloneMap)
    {
        if(!hasClone){
            bool sameness = (visualShape == collisionShape);
            if(visualShape){
                removeChild(visualShape);
                visualShape = visualShape->cloneNode(cloneMap);
                addChild(visualShape);
            }
            if(collisionShape){
                if(sameness){
                    collisionShape = visualShape;
                } else {
                    collisionShape = collisionShape->cloneNode(cloneMap);
                }
            }
            resetCollisionShapeUpdateConnection();
            hasClone = true;
            notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED);
        }
    }

    void resetCollisionShapeUpdateConnection()
    {
        if(collisionShape && collisionShape != visualShape){
            collisionShapeUpdateConnection.reset(
                collisionShape->sigUpdated().connect(
                    [this](const SgUpdate& u){
                        SgUpdate update(u);
                        notifyUpdate(update);
                    }));
        } else {
            collisionShapeUpdateConnection.disconnect();
        }
    }

    void render(SceneRenderer* renderer)
    {
        renderer->renderCustomGroup(
            this, [=](){ traverse(renderer, renderer->renderingFunctions()); });
    }

    void traverse(SceneRenderer* renderer, SceneRenderer::NodeFunctionSet* functions)
    {
        int visibility = 0;
        if(isVisible){
            static const SceneRenderer::PropertyKey key("collisionDetectionModelVisibility");
            visibility = renderer->property(key, 1);
        }
        for(auto p = cbegin(); p != cend(); ++p){
            SgNode* node = *p;
            if(node == visualShape){
                if(!(visibility & 1)){
                    continue;
                }
            }
            functions->dispatch(node);
        }
        if((visibility & 2) && (collisionShape != visualShape) && collisionShape){
            functions->dispatch(collisionShape);
        }
    }
};

typedef ref_ptr<LinkShapeGroup> LinkShapeGroupPtr;

struct NodeClassRegistration {
    NodeClassRegistration(){
        SceneNodeClassRegistry::instance().registerClass<LinkShapeGroup, SgGroup>();
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<LinkShapeGroup>(
                    [renderer](LinkShapeGroup* node){ node->render(renderer); });
            });
    }
} registration;

}

namespace cnoid {

class SceneLinkImpl
{
public:
    SceneLink* self;
    SceneBodyImpl* sceneBodyImpl;
    LinkShapeGroupPtr mainShapeGroup;
    SgGroupPtr topShapeGroup;
    bool isVisible;
    std::vector<SceneDevicePtr> sceneDevices;
    SgGroupPtr deviceGroup;
    SgTransparentGroupPtr transparentGroup;

    SceneLinkImpl(SceneLink* self, Link* link);
    void insertEffectGroup(SgGroup* effect, SgUpdateRef update);
    bool removeEffectGroup(SgGroup* parent, SgGroupPtr effect, SgUpdateRef update);
    void cloneShape(CloneMap& cloneMap);
    void setTransparency(float transparency, SgUpdateRef update = SgUpdateRef());
};

class SceneBodyImpl
{
public:
    SceneBody* self;
    SgGroupPtr sceneLinkGroup;
    std::vector<SceneDevicePtr> sceneDevices;
    std::function<SceneLink*(Link*)> sceneLinkFactory;

    SceneBodyImpl(SceneBody* self);
    void cloneShape(CloneMap& cloneMap);
};

}


SceneLink::SceneLink(SceneBody* sceneBody, Link* link)
{
    link_ = link;
    sceneBody_ = sceneBody;
    setName(link->name());
    impl = new SceneLinkImpl(this, link);
    impl->sceneBodyImpl = sceneBody_->impl;
}


SceneLinkImpl::SceneLinkImpl(SceneLink* self, Link* link)
    : self(self)
{
    mainShapeGroup = new LinkShapeGroup(link);
    topShapeGroup = mainShapeGroup;
    self->addChild(topShapeGroup);
}


SceneLink::~SceneLink()
{
    delete impl;
}


const SgNode* SceneLink::visualShape() const
{
    return impl->mainShapeGroup->visualShape;
}


SgNode* SceneLink::visualShape()
{
    return impl->mainShapeGroup->visualShape;
}


const SgNode* SceneLink::collisionShape() const
{
    return impl->mainShapeGroup->collisionShape;
}


SgNode* SceneLink::collisionShape()
{
    return impl->mainShapeGroup->collisionShape;
}


void SceneLink::insertEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    
    impl->insertEffectGroup(effect, update);
}


void SceneLinkImpl::insertEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    self->removeChild(topShapeGroup);
    effect->addChild(topShapeGroup);
    self->addChild(effect);
    topShapeGroup = effect;
    if(update){
        self->notifyUpdate(update->withAction(SgUpdate::ADDED | SgUpdate::REMOVED));
    }
}


void SceneLink::removeEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    impl->removeEffectGroup(this, effect, update);
}


bool SceneLinkImpl::removeEffectGroup(SgGroup* parent, SgGroupPtr effect, SgUpdateRef update)
{
    if(parent == mainShapeGroup){
        return false;
    }
    if(parent->removeChild(effect)){
        SgGroup* childGroup = 0;
        for(auto child : *effect){
            childGroup = dynamic_cast<SgGroup*>(child.get());
            if(childGroup){
                parent->addChild(childGroup);
                break;
            }
        }
        if(topShapeGroup == effect){
            if(childGroup){
                topShapeGroup = childGroup;
            } else {
                topShapeGroup = mainShapeGroup;
            }
        }
        effect->clearChildren();
        if(update){
            parent->notifyUpdate(update->withAction(SgUpdate::ADDED | SgUpdate::REMOVED));
        }
        return true;
    } else {
        for(auto child : *parent){
            if(auto childGroup = dynamic_cast<SgGroup*>(child.get())){
                if(removeEffectGroup(childGroup, effect, update)){
                    return true;
                }
            }
        }
    }
    return false;
}


void SceneLinkImpl::cloneShape(CloneMap& cloneMap)
{
    mainShapeGroup->cloneShapes(cloneMap);
}


void SceneLink::setVisible(bool on)
{
    impl->mainShapeGroup->setVisible(on);
}


float SceneLink::transparency() const
{
    if(!impl->transparentGroup && impl->transparentGroup->hasParents()){
        return impl->transparentGroup->transparency();
    }
    return 0.0f;
}


void SceneLink::setTransparency(float transparency, SgUpdateRef update)
{
    impl->setTransparency(transparency, update);
}


void SceneLinkImpl::setTransparency(float transparency, SgUpdateRef update)
{
    if(!transparentGroup){
        transparentGroup = new SgTransparentGroup;
        transparentGroup->setTransparency(transparency);
    } else if(transparency != transparentGroup->transparency()){
        transparentGroup->setTransparency(transparency);
        if(update){
            transparentGroup->notifyUpdate(update->withAction(SgUpdate::Modified));
        }
    }

    if(transparency > 0.0f){
        if(!transparentGroup->hasParents()){
            insertEffectGroup(transparentGroup, update);
        }
    } else {
        if(transparentGroup->hasParents()){
            self->removeEffectGroup(transparentGroup, update);
        }
    }
}


void SceneLink::makeTransparent(float transparency)
{
    SgUpdate update;
    setTransparency(transparency, update);
}


void SceneLink::addSceneDevice(SceneDevice* sdev)
{
    if(!impl->deviceGroup){
        impl->deviceGroup = new SgGroup;
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


SceneBody::SceneBody()
{
    impl = new SceneBodyImpl(this);
}


SceneBody::SceneBody(Body* body)
{
    impl = new SceneBodyImpl(this);
    setBody(body, [this](Link* link){ return new SceneLink(this, link); });
}


SceneBodyImpl::SceneBodyImpl(SceneBody* self)
    : self(self)
{
    sceneLinkGroup = new SgGroup;
}


SceneBody::~SceneBody()
{
    delete impl;
}


void SceneBody::setBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory)
{
    body_ = body;
    impl->sceneLinkFactory = sceneLinkFactory;
    addChild(impl->sceneLinkGroup);
    updateModel();
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


void SceneBody::cloneShapes(CloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->impl->cloneShape(cloneMap);
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
    if(linkIndex >= 0 && linkIndex < static_cast<int>(sceneLinks_.size())){
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


void SceneBody::setTransparency(float transparency)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->impl->setTransparency(transparency);
    }
    notifyUpdate();
}


void SceneBody::makeTransparent(float transparency, CloneMap&)
{
    setTransparency(transparency);
}


void SceneBody::makeTransparent(float transparency)
{
    setTransparency(transparency);
}


void SceneBody::insertEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    impl->sceneLinkGroup->insertChainedGroup(effect, update);
}


void SceneBody::removeEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    impl->sceneLinkGroup->removeChainedGroup(effect, update);
}
