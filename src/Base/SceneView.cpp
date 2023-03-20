/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneView.h"
#include "SceneViewConfig.h"
#include "SceneWidget.h"
#include "SceneWidgetEventHandler.h"
#include "ViewManager.h"
#include "ProjectManager.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "App.h"
#include <cnoid/SceneGraph>
#include <cnoid/ConnectionSet>
#include <QBoxLayout>
#include <list>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

vector<SceneView*> instances_;
Connection sigItemAddedConnection;
Signal<void()> sigInstancesChanged_;
Signal<void(SceneView* view)> sigLastFocusSceneViewChanged_;
SceneView* lastFocusSceneView_ = nullptr;
set<ReferencedPtr> editModeBlockRequesters;

struct SceneInfo {
    Item* item;
    RenderableItem* renderable;
    SgNodePtr node;
    bool isShown;
    ScopedConnectionSet itemConnections;
    ScopedConnection itemCheckConnection;

    SceneInfo(Item* item, RenderableItem* renderable)
        : item(item), renderable(renderable) {
        isShown = false;
    }
};

std::map<int, SceneWidgetEventHandler*> customModeIdToHandlerMap;

}

namespace cnoid {

class SceneView::Impl
{
public:
    SceneView* self;
    SceneWidget* sceneWidget;
    SgGroup* scene;
    SgUnpickableGroup* unpickableScene;
    list<SceneInfo> sceneInfos;
    SgUpdate sgUpdate;
    RootItem* rootItem;
    int itemCheckId;
    SceneViewConfigPtr config;
    bool isPendingInitialUpdate;
        
    Impl(SceneView* self);
    ~Impl();
    void onRenderableItemAdded(Item* item, RenderableItem* renderable);
    void onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter);
    void onSensitiveChanged(list<SceneInfo>::iterator infoIter, bool on);
    void showScene(list<SceneInfo>::iterator infoIter, bool show);
    void setTargetSceneItemCheckId(int id);
};

}


void SceneView::initializeClass(ExtensionManager* ext)
{
    if(instances_.empty()){
        SceneWidget::initializeClass(ext);
        
        ext->viewManager().registerClass<SceneView>(
            N_("SceneView"), N_("Scene"), ViewManager::Multiple | ViewManager::Permanent);

        sigItemAddedConnection =
            RootItem::instance()->sigItemAdded().connect(
                [](Item* item){
                    if(auto renderable = dynamic_cast<RenderableItem*>(item)){
                        for(size_t i=0; i < instances_.size(); ++i){
                            instances_[i]->impl->onRenderableItemAdded(item, renderable);
                        }
                    }
                });
    }
}


SceneView* SceneView::instance()
{
    if(!instances_.empty()){
        return instances_.front();
    }
    return nullptr;
}


std::vector<SceneView*> SceneView::instances()
{
    return instances_;
}


SignalProxy<void()> SceneView::sigInstancesChanged()
{
    return sigInstancesChanged_;
}


int SceneView::registerCustomMode(SceneWidgetEventHandler* modeHandler)
{
    int id = SceneWidget::issueUniqueCustomModeId();
    customModeIdToHandlerMap[id] = modeHandler;
    return id;
}


void SceneView::unregisterCustomMode(int id)
{
    if(id > 0){
        for(auto& instance : instances_){
            if(instance->customMode() == id){
                instance->setCustomMode(0);
            }
        }
        customModeIdToHandlerMap.erase(id);
    }
}


void SceneView::blockEditModeForAllViews(Referenced* requester)
{
    editModeBlockRequesters.insert(requester);
    for(auto& view : instances_){
        view->sceneWidget()->blockEditMode(requester);
    }
}


void SceneView::unblockEditModeForAllViews(Referenced* requester)
{
    if(editModeBlockRequesters.erase(requester) > 0){
        for(auto& view : instances_){
            view->sceneWidget()->unblockEditMode(requester);
        }
    }
}


SignalProxy<void(SceneView* view)> SceneView::sigLastFocusSceneViewChanged()
{
    return sigLastFocusSceneViewChanged_;
}


SceneView* SceneView::lastFocusSceneView()
{
    return lastFocusSceneView_;
}


static void finalizeClass()
{
    editModeBlockRequesters.clear();
    sigItemAddedConnection.disconnect();
}


SceneView::SceneView()
{
    impl = new Impl(this);

    // The following initialization must be processed after the impl has been created.
    SceneViewConfig* config = nullptr;
    if(lastFocusSceneView_){
        if(auto orgConfig = lastFocusSceneView_->impl->config){
            config = new SceneViewConfig(*orgConfig);
        }
    }
    if(!config){
        config = new SceneViewConfig;
    }

    setSceneViewConfig(config);

    if(!lastFocusSceneView_){
        onFocusChanged(true);
    }
}


/**
   The constructor used by an inherited view class.
   Note that a SceneViewConfig instance must be set using the setSceneViewConfig function
   in the constructor of the inherited clasls.
*/
SceneView::SceneView(NoSceneViewConfig_t)
{
    impl = new Impl(this);

    sigInstancesChanged_();

    if(!lastFocusSceneView_){
        onFocusChanged(true);
    }
}


SceneView::Impl::Impl(SceneView* self)
    : self(self)
{
    self->setDefaultLayoutArea(CenterArea);
    
    sceneWidget = new SceneWidget(self);
    sceneWidget->setModeSyncEnabled(true);
    //sceneWidget->activate();
    scene = sceneWidget->scene();
    sceneWidget->setObjectName(self->windowTitle());
    self->sigWindowTitleChanged().connect(
        [this](string title){ sceneWidget->setObjectName(title.c_str()); });
    unpickableScene = new SgUnpickableGroup;
    scene->addChild(unpickableScene);

    for(auto& requester : editModeBlockRequesters){
        sceneWidget->blockEditMode(requester);
    }

    auto vbox = new QVBoxLayout;
    vbox->addWidget(sceneWidget);
    self->setLayout(vbox);

    rootItem = RootItem::instance();
    itemCheckId = Item::PrimaryCheck;

    if(!instances_.empty()){
        auto mainImpl = instances_.front()->impl;
        for(auto& info : mainImpl->sceneInfos){
            onRenderableItemAdded(info.item, info.renderable);
        }
    }

    instances_.push_back(self);

    isPendingInitialUpdate = false;
}


void SceneView::setSceneViewConfig(SceneViewConfig* config)
{
    impl->config = config;
    
    // Flag to avoid redundant initialization
    bool doUpdate = !ProjectManager::instance()->isLoadingProject() && !App::isDoingInitialization();
    impl->isPendingInitialUpdate = !doUpdate;
    config->addSceneView(this, doUpdate);
}


SceneView::~SceneView()
{
    delete impl;
}


SceneView::Impl::~Impl()
{
    if(config){
        config->removeSceneView(self);
    }
    
    instances_.erase(std::find(instances_.begin(), instances_.end(), self));

    if(lastFocusSceneView_ == self){
        if(instances_.empty()){
            lastFocusSceneView_ = nullptr;
        } else {
            lastFocusSceneView_ = instances_.front();
        }
        sigLastFocusSceneViewChanged_(lastFocusSceneView_);
    }

    sigInstancesChanged_();
    
    if(instances_.empty()){
        finalizeClass();
    }
}


SceneViewConfig* SceneView::sceneViewConfig()
{
    return impl->config;
}


SceneWidget* SceneView::sceneWidget()
{
    return impl->sceneWidget;
}


SceneRenderer* SceneView::renderer()
{
    return impl->sceneWidget->renderer();
}


void SceneView::onFocusChanged(bool on)
{
    if(on){
        if(this != lastFocusSceneView_){
            lastFocusSceneView_ = this;
            sigLastFocusSceneViewChanged_(this);
        }
    }
}


SgGroup* SceneView::scene()
{
    return impl->scene;
}


SgGroup* SceneView::systemNodeGroup()
{
    return impl->sceneWidget->systemNodeGroup();
}


bool SceneView::setCustomMode(int mode)
{
    bool isValid = true;
    if(mode != customMode()){
        auto p = customModeIdToHandlerMap.find(mode);
        if(p != customModeIdToHandlerMap.end()){
            impl->sceneWidget->activateCustomMode(p->second, mode);
        } else {
            impl->sceneWidget->deactivateCustomMode();
            isValid = false;
        }
    }
    return isValid;
}


int SceneView::customMode() const
{
    return impl->sceneWidget->activeCustomMode();
}


void SceneView::Impl::onRenderableItemAdded(Item* item, RenderableItem* renderable)
{
    sceneInfos.emplace_back(item, renderable);
    list<SceneInfo>::iterator infoIter = sceneInfos.end();
    --infoIter;
    SceneInfo& info = *infoIter;
        
    info.itemConnections.add(
        item->sigDisconnectedFromRoot().connect(
            [this, infoIter](){ onRenderableItemDisconnectedFromRoot(infoIter); }));

    info.itemConnections.add(
        renderable->sigSceneSensitiveChanged().connect(
            [this, infoIter](bool on){ onSensitiveChanged(infoIter, on); }));

    info.itemCheckConnection =
        item->sigCheckToggled(itemCheckId).connect(
            [this, infoIter](bool on){ showScene(infoIter, on); });
        
    if(item->isChecked(itemCheckId)){
        showScene(infoIter, true);
    }
}


void SceneView::Impl::onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter)
{
    showScene(infoIter, false);
    sceneInfos.erase(infoIter);
}


void SceneView::Impl::onSensitiveChanged(list<SceneInfo>::iterator infoIter, bool on)
{
    if(infoIter->isShown){
        if(auto node = infoIter->node){
            if(on){
                unpickableScene->removeChild(node, sgUpdate);
                scene->addChild(node, sgUpdate);
            } else {
                scene->removeChild(node, sgUpdate);
                unpickableScene->addChild(node, sgUpdate);
            }
        }
    }
}


void SceneView::Impl::showScene(list<SceneInfo>::iterator infoIter, bool show)
{
    if(infoIter->isShown && !show){
        if(auto node = infoIter->node){
            if(infoIter->renderable->isSceneSensitive()){
                scene->removeChild(node, sgUpdate);
            } else {
                unpickableScene->removeChild(node, sgUpdate);
            }
        }
        infoIter->isShown = false;
        
    } else if(!infoIter->isShown && show){
        if(!infoIter->node){
            infoIter->node = infoIter->renderable->getScene();
        }
        if(auto node = infoIter->node){
            if(infoIter->renderable->isSceneSensitive()){
                scene->addChild(node, sgUpdate);
            } else {
                unpickableScene->addChild(node, sgUpdate);
            }
            infoIter->isShown = true;
        }
    }
}


QWidget* SceneView::indicatorOnInfoBar()
{
    return impl->sceneWidget->indicator();
}


void SceneView::setTargetSceneItemCheckId(int id)
{
    impl->setTargetSceneItemCheckId(id);
}


void SceneView::Impl::setTargetSceneItemCheckId(int id)
{
    if(id != itemCheckId){
        itemCheckId = id;
        for(auto it = sceneInfos.begin(); it != sceneInfos.end(); ++it){
            it->itemCheckConnection =
                it->item->sigCheckToggled(id).connect(
                    [this, it](bool on) { showScene(it, on); });
            showScene(it, it->item->isChecked(id));
        }
    }
}


bool SceneView::storeState(Archive& archive)
{
    bool result = impl->sceneWidget->storeState(archive);
    return impl->config->store(&archive) && result;
}


bool SceneView::restoreState(const Archive& archive)
{
    bool result = impl->config->restore(&archive);
    result &= impl->sceneWidget->restoreState(archive);
    return result;
}


void SceneView::onRestored(bool stateRestored)
{
    if(impl->isPendingInitialUpdate){
        if(!stateRestored){
            impl->config->updateSceneViews();
        }
        impl->isPendingInitialUpdate = false;
    }
}
