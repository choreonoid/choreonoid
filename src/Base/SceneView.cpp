/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneView.h"
#include "SceneWidget.h"
#include "SceneWidgetEventHandler.h"
#include "ViewManager.h"
#include "Separator.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "Buttons.h"
#include "CheckBox.h"
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
    CheckBox dedicatedCheckCheck;
    int dedicatedCheckId;
        
    Impl(SceneView* self);
    ~Impl();
    void onRenderableItemAdded(Item* item, RenderableItem* renderable);
    void onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter);
    void onDedicatedCheckToggled(bool on);
    void onSensitiveChanged(list<SceneInfo>::iterator infoIter, bool on);
    void showScene(list<SceneInfo>::iterator infoIter, bool show);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void SceneView::initializeClass(ExtensionManager* ext)
{
    if(instances_.empty()){
        SceneWidget::initializeClass(ext);
        
        ext->viewManager().registerClass<SceneView>(
            "SceneView", N_("Scene"), ViewManager::MULTI_DEFAULT);

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


SignalProxy<void(SceneView* view)> SceneView::sigLastFocusViewChanged()
{
    return sigLastFocusViewChanged_;
}


static void finalizeClass()
{
    sigItemAddedConnection.disconnect();
}


SceneView::SceneView()
{
    impl = new Impl(this);
}


SceneView::Impl::Impl(SceneView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::RIGHT);
    
    sceneWidget = new SceneWidget(self);
    sceneWidget->setModeSyncEnabled(true);
    //sceneWidget->activate();
    scene = sceneWidget->scene();
    sceneWidget->setObjectName(self->windowTitle());
    self->sigWindowTitleChanged().connect(
        [this](string title){ sceneWidget->setObjectName(title.c_str()); });
    unpickableScene = new SgUnpickableGroup;
    scene->addChild(unpickableScene);

    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->addWidget(sceneWidget);
    self->setLayout(vbox);

    vbox = sceneWidget->configDialogVBox();
    //vbox->addWidget(new HSeparator);
    QHBoxLayout* hbox = new QHBoxLayout;
    dedicatedCheckCheck.setText(_("Use dedicated item tree view checks to select the target items"));
    dedicatedCheckCheck.sigToggled().connect(
        [&](bool on){ onDedicatedCheckToggled(on); });
    hbox->addWidget(&dedicatedCheckCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    rootItem = RootItem::instance();

    dedicatedCheckId = -1;

    if(!instances_.empty()){
        auto mainImpl = instances_.front()->impl;
        list<SceneInfo>::iterator p;
        for(p = mainImpl->sceneInfos.begin(); p != mainImpl->sceneInfos.end(); ++p){
            SceneInfo& info = *p;
            onRenderableItemAdded(info.item, info.renderable);
        }
    }

    instances_.push_back(self);
}


SceneView::~SceneView()
{
    delete impl;
}


SceneView::Impl::~Impl()
{
    if(dedicatedCheckId >= 0){
        rootItem->releaseCheckEntry(dedicatedCheckId);
    }

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


void SceneView::onFocusChanged(bool on)
{
    if(on){
        lastFocusView_ = this;
        sigLastFocusViewChanged_(this);
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

    int checkId = dedicatedCheckCheck.isChecked() ? dedicatedCheckId : Item::PrimaryCheck;
        
    info.itemCheckConnection =
        item->sigCheckToggled(checkId).connect(
            [this, infoIter](bool on){ showScene(infoIter, on); });
        
    if(item->isChecked(checkId)){
        showScene(infoIter, true);
    }
}


void SceneView::Impl::onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter)
{
    showScene(infoIter, false);
    sceneInfos.erase(infoIter);
}


void SceneView::Impl::onDedicatedCheckToggled(bool on)
{
    int checkId = Item::PrimaryCheck;
    
    if(on){
        if(dedicatedCheckId < 0){
            dedicatedCheckId = rootItem->addCheckEntry(self->windowTitle().toStdString());
        }
        checkId = dedicatedCheckId;

    } else {
        if(dedicatedCheckId >= 0){
            rootItem->releaseCheckEntry(dedicatedCheckId);
            dedicatedCheckId = -1;
        }
    }

    for(list<SceneInfo>::iterator p = sceneInfos.begin(); p != sceneInfos.end(); ++p){
        p->itemCheckConnection.disconnect();
        p->itemCheckConnection =
            p->item->sigCheckToggled(checkId).connect(
                [this, p](bool on) { showScene(p, on); });
        
        showScene(p, p->item->isChecked(checkId));
    }
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


bool SceneView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool SceneView::Impl::storeState(Archive& archive)
{
    bool result = true;
    result &= sceneWidget->storeState(archive);
    archive.write("isDedicatedItemCheckEnabled", dedicatedCheckCheck.isChecked());
    if(dedicatedCheckCheck.isChecked()){
        rootItem->storeCheckStates(dedicatedCheckId, archive, "checked");
    }
    return result;
}


bool SceneView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool SceneView::Impl::restoreState(const Archive& archive)
{
    bool result = sceneWidget->restoreState(archive);

    bool isDedicatedItemCheckEnabled = false;
    if(archive.read("isDedicatedItemCheckEnabled", isDedicatedItemCheckEnabled) ||
       archive.read("dedicatedItemTreeViewChecks", isDedicatedItemCheckEnabled) /* old format */){
        dedicatedCheckCheck.setChecked(isDedicatedItemCheckEnabled);
    }

    if(dedicatedCheckId >= 0){
        ref_ptr<const Archive> pArchive = &archive;
        archive.addPostProcess(
            [this, pArchive](){
                rootItem->restoreCheckStates(dedicatedCheckId, *pArchive, "checked");
            });
    }
    
    return result;
}
