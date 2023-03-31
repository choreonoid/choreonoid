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
    SceneLink::Impl* sceneLinkImpl;
    SgNodePtr visualShape;
    SgNodePtr collisionShape;
    ScopedConnection collisionShapeUpdateConnection;
    bool hasClone;
    
    LinkShapeGroup(SceneLink::Impl* sceneLinkImpl, Link* link);
    void cloneShapes(CloneMap& cloneMap);
    void resetCollisionShapeUpdateConnection();
    void render(SceneRenderer* renderer);
    void traverse(SceneRenderer* renderer, SceneRenderer::NodeFunctionSet* functions);
};

typedef ref_ptr<LinkShapeGroup> LinkShapeGroupPtr;

struct NodeClassRegistration {
    NodeClassRegistration(){
        SceneNodeClassRegistry::instance().registerClass<LinkShapeGroup, SgGroup>("LinkShapeGroup");
        SceneRenderer::addExtension(
            [](SceneRenderer* renderer){
                renderer->renderingFunctions()->setFunction<LinkShapeGroup>(
                    [renderer](LinkShapeGroup* node){ node->render(renderer); });
            });
    }
} registration;

}

namespace cnoid {

class SceneLink::Impl
{
public:
    SceneLink* self;
    SceneBody::Impl* sceneBodyImpl;
    LinkShapeGroupPtr mainShapeGroup;
    SgGroupPtr topShapeGroup;
    bool isVisible;
    std::vector<SceneDevicePtr> sceneDevices;
    SgGroupPtr deviceGroup;
    SgTransparentGroupPtr transparentGroup;

    Impl(SceneLink* self, Link* link);
    void insertEffectGroup(SgGroup* effect, SgUpdateRef& update);
    bool removeEffectGroup(SgGroup* parent, SgGroupPtr effect, SgUpdateRef& update);
    void cloneShape(CloneMap& cloneMap);
    void clearSceneDevices();
    void setTransparency(float transparency, SgUpdateRef update = nullptr);
};

class SceneBody::Impl
{
public:
    SceneBody* self;
    SgGroupPtr sceneLinkGroup;
    SgGroupPtr lastEffectGroup;
    std::vector<SceneDevicePtr> sceneDevices;
    std::function<SceneLink*(Link*)> sceneLinkFactory;
    SgGroupPtr multiplexSceneBodyGroup;
    std::vector<SceneBodyPtr> multiplexSceneBodies;
    std::vector<SceneBodyPtr> multiplexSceneBodyCache;
    ScopedConnection existenceConnection;

    Impl(SceneBody* self);
    void updateLinkPositions(Body* body, vector<SceneLinkPtr>& sceneLinks, SgUpdateRef& update);
    void updateMultiplexBodyPositions(SgUpdateRef& update);
    SceneBody* addMultiplexSceneBody(SgUpdateRef& update);
    void removeSubsequentMultiplexSceneBodies(int index, SgUpdateRef& update, bool doCache);
    void clearSceneDevices();    
    void onBodyExistenceChanged(bool on);
};

}


SceneLink::SceneLink(SceneBody* sceneBody, Link* link)
{
    link_ = link;
    sceneBody_ = sceneBody;
    setName(link->name());
    impl = new Impl(this, link);
    impl->sceneBodyImpl = sceneBody_->impl;
}


SceneLink::Impl::Impl(SceneLink* self, Link* link)
    : self(self)
{
    mainShapeGroup = new LinkShapeGroup(this, link);
    topShapeGroup = mainShapeGroup;
    isVisible = true;
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


void SceneLink::Impl::insertEffectGroup(SgGroup* effect, SgUpdateRef& update)
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


bool SceneLink::Impl::removeEffectGroup(SgGroup* parent, SgGroupPtr effect, SgUpdateRef& update)
{
    if(parent == mainShapeGroup){
        return false;
    }
    if(parent->removeChild(effect)){
        SgGroup* childGroup = nullptr;
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


void SceneLink::Impl::cloneShape(CloneMap& cloneMap)
{
    mainShapeGroup->cloneShapes(cloneMap);
}


void SceneLink::setVisible(bool on)
{
    impl->isVisible = on;
}


bool SceneLink::isVisible() const
{
    return impl->isVisible;
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
    return nullptr;
}


void SceneLink::Impl::clearSceneDevices()
{
    sceneDevices.clear();
    if(deviceGroup){
        deviceGroup->clearChildren();
    }
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


void SceneLink::Impl::setTransparency(float transparency, SgUpdateRef update)
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


LinkShapeGroup::LinkShapeGroup(SceneLink::Impl* sceneLinkImpl, Link* link)
    : SgGroup(findClassId<LinkShapeGroup>()),
      sceneLinkImpl(sceneLinkImpl)
{
    visualShape = link->visualShape();
    if(visualShape){
        addChild(visualShape);
    }
    collisionShape = link->collisionShape();
    resetCollisionShapeUpdateConnection();

    hasClone = false;
}


void LinkShapeGroup::cloneShapes(CloneMap& cloneMap)
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


void LinkShapeGroup::resetCollisionShapeUpdateConnection()
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


void LinkShapeGroup::render(SceneRenderer* renderer)
{
    renderer->renderCustomGroup(
        this, [=](){ traverse(renderer, renderer->renderingFunctions()); });
}


void LinkShapeGroup::traverse(SceneRenderer* renderer, SceneRenderer::NodeFunctionSet* functions)
{
    if(sceneLinkImpl->isVisible){
        static const SceneRenderer::PropertyKey key("collisionDetectionModelVisibility");
        int visibility = renderer->property(key, 1);

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
}


SceneBody::SceneBody()
{
    impl = new Impl(this);
}


SceneBody::SceneBody(Body* body)
{
    impl = new Impl(this);
    setBody(body, [this](Link* link){ return new SceneLink(this, link); });
}


SceneBody::Impl::Impl(SceneBody* self)
    : self(self)
{
    sceneLinkGroup = new SgGroup;
    lastEffectGroup = sceneLinkGroup;
    self->addChild(sceneLinkGroup);
}


SceneBody::~SceneBody()
{
    delete impl;
}


void SceneBody::setBody(Body* body, std::function<SceneLink*(Link*)> sceneLinkFactory)
{
    body_ = body;
    impl->sceneLinkFactory = sceneLinkFactory;
    updateSceneModel();

    impl->existenceConnection =
        body->sigExistenceChanged().connect([this](bool on){ impl->onBodyExistenceChanged(on); });
}


void SceneBody::updateSceneModel()
{
    setName(body_->name());

    if(!sceneLinks_.empty()){
        impl->lastEffectGroup->clearChildren();
        sceneLinks_.clear();
    }

    SgUpdateRef noUpdate;
    impl->removeSubsequentMultiplexSceneBodies(0, noUpdate, false);

    const int n = body_->numLinks();
    for(int i=0; i < n; ++i){
        Link* link = body_->link(i);
        SceneLink* sLink = impl->sceneLinkFactory(link);
        impl->lastEffectGroup->addChild(sLink);
        sceneLinks_.push_back(sLink);
    }
    impl->updateLinkPositions(body_, sceneLinks_, noUpdate);

    updateSceneDeviceModels(false);
    
    notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED);
}


void SceneBody::updateSceneDeviceModels(bool doNotify)
{
    impl->clearSceneDevices();
    for(auto& device : body_->devices()){
        if(auto sceneDevice = SceneDevice::create(device)){
            sceneLinks_[device->link()->index()]->addSceneDevice(sceneDevice);
            impl->sceneDevices.push_back(sceneDevice);
        }
    }
    updateSceneDevices(0.0);

    if(doNotify){
        notifyUpdate(SgUpdate::REMOVED | SgUpdate::ADDED | SgUpdate::MODIFIED);
    }
}


void SceneBody::cloneShapes(CloneMap& cloneMap)
{
    for(size_t i=0; i < sceneLinks_.size(); ++i){
        sceneLinks_[i]->impl->cloneShape(cloneMap);
    }
}


void SceneBody::updateLinkPositions(SgUpdateRef update)
{
    // Main body
    impl->updateLinkPositions(body_, sceneLinks_, update);

    impl->updateMultiplexBodyPositions(update);
}


void SceneBody::Impl::updateLinkPositions(Body* body, vector<SceneLinkPtr>& sceneLinks, SgUpdateRef& update)
{
    int n = std::min(body->numLinks(), static_cast<int>(sceneLinks.size()));
    for(int i=0; i < n; ++i){
        SceneLink* sceneLink = sceneLinks[i];
        Link* link = body->link(i);
        sceneLink->setPosition(link->position());
        if(update){
            sceneLink->notifyUpdate(*update);
        }
    }
}


void SceneBody::Impl::updateMultiplexBodyPositions(SgUpdateRef& update)
{
    auto multiplexBody = self->body_->nextMultiplexBody();
    int multiplexBodyIndex = 0;
    while(multiplexBody){
        SceneBody* sceneBody;
        if(multiplexBodyIndex < multiplexSceneBodies.size()){
            sceneBody = multiplexSceneBodies[multiplexBodyIndex];
        } else {
            sceneBody = addMultiplexSceneBody(update);
        }
        auto& sceneLinks = sceneBody->sceneLinks_;
        updateLinkPositions(multiplexBody, sceneLinks, update);
        multiplexBody = multiplexBody->nextMultiplexBody();
        ++multiplexBodyIndex;
    }
    removeSubsequentMultiplexSceneBodies(multiplexBodyIndex, update, true);
}


SceneBody* SceneBody::Impl::addMultiplexSceneBody(SgUpdateRef& update)
{
    SceneBodyPtr sceneBody;

    if(multiplexSceneBodyCache.empty()){
        sceneBody = new SceneBody(self->body_);
        //sceneBody = new SceneBody(*self);
    } else {
        sceneBody = multiplexSceneBodyCache.back();
        multiplexSceneBodyCache.pop_back();
    }
    multiplexSceneBodies.push_back(sceneBody);

    if(!multiplexSceneBodyGroup){
        multiplexSceneBodyGroup = new SgGroup;
        self->addChild(multiplexSceneBodyGroup);
    }
    multiplexSceneBodyGroup->addChild(sceneBody, update);

    return sceneBody;
}


void SceneBody::Impl::removeSubsequentMultiplexSceneBodies(int index, SgUpdateRef& update, bool doCache)
{
    int n = multiplexSceneBodies.size();
    if(index < n){
        for(size_t i = index; i < n; ++i){
            multiplexSceneBodyGroup->removeChildAt(index, update);
            if(doCache){
                multiplexSceneBodyCache.push_back(multiplexSceneBodies[i]);
            }
        }
        multiplexSceneBodies.resize(index);
    }
}


void SceneBody::Impl::clearSceneDevices()
{
    sceneDevices.clear();
    for(auto& sceneLink : self->sceneLinks_){
        sceneLink->impl->clearSceneDevices();
    }
}


SceneDevice* SceneBody::getSceneDevice(Device* device)
{
    const int linkIndex = device->link()->index();
    if(linkIndex >= 0 && linkIndex < static_cast<int>(sceneLinks_.size())){
        return sceneLinks_[linkIndex]->getSceneDevice(device);
    }
    return nullptr;
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


void SceneBody::Impl::onBodyExistenceChanged(bool on)
{
    if(on){
        self->addChildOnce(sceneLinkGroup);
        if(multiplexSceneBodyGroup){
            self->addChildOnce(multiplexSceneBodyGroup);
        }
        self->notifyUpdate(SgUpdate::ADDED);
    } else {
        self->removeChild(sceneLinkGroup);
        if(multiplexSceneBodyGroup){
            self->removeChild(multiplexSceneBodyGroup);
        }
        self->notifyUpdate(SgUpdate::REMOVED);
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
    if(impl->lastEffectGroup == impl->sceneLinkGroup){
        impl->lastEffectGroup = effect;
    }
}


void SceneBody::removeEffectGroup(SgGroup* effect, SgUpdateRef update)
{
    if(effect == impl->lastEffectGroup){
        auto group = impl->sceneLinkGroup;
        while(group){
            if(group->empty()){
                break;
            }
            auto childGroup = dynamic_cast<SgGroup*>(group->child(0));
            if(!childGroup){
                break;
            }
            if(childGroup == effect){
                impl->lastEffectGroup = group;
                break;
            }
            group = childGroup;
        }
    }
    impl->sceneLinkGroup->removeChainedGroup(effect, update);
}
