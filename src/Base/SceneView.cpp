/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneView.h"
#include "SceneViewConfig.h"
#include "SceneWidget.h"
#include "SceneWidgetEventHandler.h"
#include "ViewManager.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include <cnoid/SceneGraph>
#include <cnoid/ConnectionSet>
#include <list>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

vector<SceneView*> instances_;
Connection sigItemAddedConnection;
Signal<void(SceneView* view)> sigLastFocusViewChanged_;
SceneView* lastFocusView_ = nullptr;
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
    SceneViewConfig* config;
        
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


int SceneView::registerCustomMode(SceneWidgetEventHandler* modeHandler)
{
    int id = SceneWidget::issueUniqueCustomModeId();
    customModeIdToHandlerMap[id] = modeHandler;
    return id;
}


void SceneView::unregisterCustomMode(int id)
{
    for(auto& instance : instances_){
        if(instance->customMode() == id){
            instance->setCustomMode(0);
        }
    }
    customModeIdToHandlerMap.erase(id);
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


SignalProxy<void(SceneView* view)> SceneView::sigLastFocusViewChanged()
{
    return sigLastFocusViewChanged_;
}


static void finalizeClass()
{
    editModeBlockRequesters.clear();
    sigItemAddedConnection.disconnect();
}


SceneView::SceneView()
{
    impl = new Impl(this);


    if(instances_.empty()){
        impl->config = new SceneViewConfig(this);
        lastFocusView_ = this;

    } else if(lastFocusView_){
        impl->config = new SceneViewConfig(*lastFocusView_->impl->config, this);
    }

    instances_.push_back(this);
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

    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->addWidget(sceneWidget);
    self->setLayout(vbox);

    rootItem = RootItem::instance();
    itemCheckId = Item::PrimaryCheck;

    if(!instances_.empty()){
        auto mainImpl = instances_.front()->impl;
        list<SceneInfo>::iterator p;
        for(p = mainImpl->sceneInfos.begin(); p != mainImpl->sceneInfos.end(); ++p){
            SceneInfo& info = *p;
            onRenderableItemAdded(info.item, info.renderable);
        }
    }
}


SceneView::~SceneView()
{
    delete impl;
}


SceneView::Impl::~Impl()
{
    delete config;

    instances_.erase(std::find(instances_.begin(), instances_.end(), self));

    if(lastFocusView_ == self){
        if(instances_.empty()){
            lastFocusView_ = nullptr;
        } else {
            lastFocusView_ = instances_.front();
        }
        sigLastFocusViewChanged_(lastFocusView_);
    }
    
    if(instances_.empty()){
        finalizeClass();
    }
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
        if(this != lastFocusView_){
            lastFocusView_ = this;
            sigLastFocusViewChanged_(this);
        }
    }
}
    

SgGroup* SceneView::scene()
{
    return impl->scene;
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


void SceneView::showConfigDialog()
{
    impl->config->showConfigDialog();
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
