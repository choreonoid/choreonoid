/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneView.h"
#include "SceneWidget.h"
#include "ViewManager.h"
#include "Separator.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "Buttons.h"
#include "CheckBox.h"
#include <cnoid/SceneGraph>
#include <list>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
vector<SceneView*> instances;
Connection sigItemAddedConnection;
}

namespace cnoid {

class SceneViewImpl
{
public:
    SceneView* self;
    SceneWidget* sceneWidget;
    SgGroup* scene;

    struct SceneInfo {
        Item* item;
        RenderableItem* renderable;
        SgNodePtr scene;
        bool isShown;
        Connection sigDisconnectedFromRootConnection;
        Connection sigCheckToggledConnection;
        SceneInfo(Item* item, RenderableItem* renderable)
            : item(item), renderable(renderable) {
            isShown = false;
        }
        ~SceneInfo(){
            sigDisconnectedFromRootConnection.disconnect();
            sigCheckToggledConnection.disconnect();
        }
    };

    list<SceneInfo> sceneInfos;

    RootItem* rootItem;
    CheckBox dedicatedCheckCheck;
    int dedicatedCheckId;
        
    SceneViewImpl(SceneView* self);
    ~SceneViewImpl();
    void onRenderableItemAdded(Item* item, RenderableItem* renderable);
    void onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter);
    void showScene(list<SceneInfo>::iterator infoIter, bool show);
    void onDedicatedCheckToggled(bool on);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreDedicatedItemChecks(const Archive& archive);
};

}


SceneView* SceneView::instance()
{
    if(instances.empty()){
        return 0;
    }
    return instances.front();
}


void SceneView::initializeClass(ExtensionManager* ext)
{
    if(instances.empty()){
        SceneWidget::initializeClass(ext);
        
        ext->viewManager().registerClass<SceneView>(
            "SceneView", N_("Scene"), ViewManager::MULTI_DEFAULT);

        sigItemAddedConnection =
            RootItem::instance()->sigItemAdded().connect(
                [](Item* item){ SceneView::onItemAdded(item); });
    }
}


namespace {

void finalizeClass()
{
    sigItemAddedConnection.disconnect();
}

}


SceneView::SceneView()
{
    impl = new SceneViewImpl(this);
}


SceneViewImpl::SceneViewImpl(SceneView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::RIGHT);
    
    sceneWidget = new SceneWidget;
    scene = sceneWidget->scene();
    sceneWidget->setObjectName(self->windowTitle());
    self->sigWindowTitleChanged().connect(
        [this](string title){ sceneWidget->setObjectName(title.c_str()); });

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

    if(!instances.empty()){
        SceneViewImpl* mainImpl = instances.front()->impl;
        list<SceneInfo>::iterator p;
        for(p = mainImpl->sceneInfos.begin(); p != mainImpl->sceneInfos.end(); ++p){
            SceneInfo& info = *p;
            onRenderableItemAdded(info.item, info.renderable);
        }
    }
    
    instances.push_back(self);
}


SceneView::~SceneView()
{
    delete impl;
}


SceneViewImpl::~SceneViewImpl()
{
    if(dedicatedCheckId >= 0){
        rootItem->releaseCheckEntry(dedicatedCheckId);
    }

    instances.erase(std::find(instances.begin(), instances.end(), self));

    if(instances.empty()){
        finalizeClass();
    }
}


void SceneView::onActivated()
{

}


void SceneView::onDeactivated()
{

}


SceneWidget* SceneView::sceneWidget()
{
    return impl->sceneWidget;
}
    

SgGroup* SceneView::scene()
{
    return impl->scene;
}


void SceneView::onItemAdded(Item* item)
{
    if(RenderableItem* renderable = dynamic_cast<RenderableItem*>(item)){
        for(size_t i=0; i < instances.size(); ++i){
            instances[i]->impl->onRenderableItemAdded(item, renderable);
        }
    }
}


void SceneViewImpl::onRenderableItemAdded(Item* item, RenderableItem* renderable)
{
    sceneInfos.push_back(SceneInfo(item, renderable));
    list<SceneInfo>::iterator infoIter = sceneInfos.end();
    --infoIter;
    SceneInfo& info = *infoIter;
        
    info.sigDisconnectedFromRootConnection =
        item->sigDisconnectedFromRoot().connect(
            [this, infoIter](){ onRenderableItemDisconnectedFromRoot(infoIter); });

    int checkId = dedicatedCheckCheck.isChecked() ? dedicatedCheckId : Item::PrimaryCheck;
        
    info.sigCheckToggledConnection =
        item->sigCheckToggled(checkId).connect(
            [this, infoIter](bool on){ showScene(infoIter, on); });
        
    if(item->isChecked(checkId)){
        showScene(infoIter, true);
    }
}


void SceneViewImpl::onRenderableItemDisconnectedFromRoot(list<SceneInfo>::iterator infoIter)
{
    showScene(infoIter, false);
    sceneInfos.erase(infoIter);
}


void SceneViewImpl::showScene(list<SceneInfo>::iterator infoIter, bool show)
{
    if(infoIter->isShown && !show){
        if(infoIter->scene){
            scene->removeChild(infoIter->scene, true);
        }
        infoIter->isShown = false;
        
    } else if(!infoIter->isShown && show){
        if(!infoIter->scene){
            infoIter->scene = infoIter->renderable->getScene();
        }
        if(infoIter->scene){
            scene->addChild(infoIter->scene, true);
            infoIter->isShown = true;
        }
    }
}


void SceneViewImpl::onDedicatedCheckToggled(bool on)
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
        p->sigCheckToggledConnection.disconnect();
        p->sigCheckToggledConnection =
            p->item->sigCheckToggled(checkId).connect(
                [this, p](bool on) { showScene(p, on); });
        
        showScene(p, p->item->isChecked(checkId));
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


bool SceneViewImpl::storeState(Archive& archive)
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


bool SceneViewImpl::restoreState(const Archive& archive)
{
    bool result = sceneWidget->restoreState(archive);

    bool isDedicatedItemCheckEnabled = false;
    if(archive.read("isDedicatedItemCheckEnabled", isDedicatedItemCheckEnabled) ||
       archive.read("dedicatedItemTreeViewChecks", isDedicatedItemCheckEnabled) /* old format */){
        dedicatedCheckCheck.setChecked(isDedicatedItemCheckEnabled);
        archive.addPostProcess([&](){ restoreDedicatedItemChecks(archive); });
    }
    
    return result;
}


void SceneViewImpl::restoreDedicatedItemChecks(const Archive& archive)
{
    rootItem->restoreCheckStates(dedicatedCheckId, archive, "checked");
}
