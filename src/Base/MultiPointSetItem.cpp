/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiPointSetItem.h"
#include "SceneWidgetEditable.h"
#include "MenuManager.h"
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/SceneMarker>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {

class SceneMultiPointSet : public SgPosTransform, public SceneWidgetEditable
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    weak_ref_ptr<MultiPointSetItem> weakMultiPointSetItem;
    bool isEditable;
    SgGroupPtr pointSetGroup;
    SgGroupPtr attentionPointMarkerGroup;
    Signal<void(const Affine3& T)> sigOffsetTransformChanged;
    Signal<void()> sigAttentionPointsChanged;

    SceneMultiPointSet(MultiPointSetItemImpl* multiPointSetItem);

    int numAttentionPoints() const;
    Vector3 attentionPoint(int index) const;
    void clearAttentionPoints(bool doNotify);
    void addAttentionPoint(const Vector3& point, bool doNotify);
    void setAttentionPoint(const Vector3& p, bool doNotify);
    bool removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify);
    void notifyAttentionPointChange();

    virtual bool onButtonPressEvent(const SceneWidgetEvent& event);
    virtual bool onPointerMoveEvent(const SceneWidgetEvent& event);
    virtual void onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager);
    virtual void onSceneModeChanged(const SceneWidgetEvent& event);

    //void startEraserMode(SceneWidget* sceneWidget);
};

typedef ref_ptr<SceneMultiPointSet> SceneMultiPointSetPtr;

}

namespace cnoid {
        
class MultiPointSetItemImpl
{
public:
    MultiPointSetItem* self;

    struct PointSetItemInfo : public Referenced {
        int index;
        ScopedConnection pointSetUpdateConnection;
            
    };
    typedef ref_ptr<PointSetItemInfo> PointSetItemInfoPtr;

    typedef map<PointSetItem*, PointSetItemInfoPtr> ItemInfoMap;
    ItemInfoMap itemInfoMap;

    ItemList<PointSetItem> pointSetItems;
    ItemList<PointSetItem> selectedPointSetItems;
    ItemList<PointSetItem> visiblePointSetItems;
    SceneMultiPointSetPtr scene;
    Selection renderingMode;
    double pointSize;
    double voxelSize;
    string directory;
    ScopedConnection itemSelectionChangedConnection;
    ScopedConnection subTreeChangedConnection;
    Signal<void(int index)> sigPointSetItemAdded;
    Signal<void(int index)> sigPointSetItemUpdated;

    MultiPointSetItemImpl(MultiPointSetItem* self);
    MultiPointSetItemImpl(MultiPointSetItem* self, const MultiPointSetItemImpl& org);
    void initialize();
    void onItemSelectionChanged(ItemList<PointSetItem> items);
    void onSubTreeChanged();
    void onPointSetUpdated(PointSetItem* item);
    void setRenderingMode(int mode);
    void setPointSize(double size);
    void setVoxelSize(double size);
    void selectSinglePointSetItem(int index);
    bool onRenderingModePropertyChanged(int mode);
};

}


void MultiPointSetItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<MultiPointSetItem>(N_("MultiPointSetItem"));
        im.addCreationPanel<MultiPointSetItem>();
        initialized = true;
    }
}


MultiPointSetItem::MultiPointSetItem()
{
    impl = new MultiPointSetItemImpl(this);
}


MultiPointSetItemImpl::MultiPointSetItemImpl(MultiPointSetItem* self)
    : self(self)
{
    initialize();
    renderingMode.select(PointSetItem::POINT);

}


MultiPointSetItem::MultiPointSetItem(const MultiPointSetItem& org)
    : Item(org)
{
    impl = new MultiPointSetItemImpl(this, *org.impl);
}


MultiPointSetItemImpl::MultiPointSetItemImpl(MultiPointSetItem* self, const MultiPointSetItemImpl& org)
    : self(self)
{
    initialize();
    renderingMode.select(org.renderingMode.which());
}


void MultiPointSetItemImpl::initialize()
{
    scene = new SceneMultiPointSet(this);
    
    renderingMode.setSymbol(PointSetItem::POINT, N_("Point"));
    renderingMode.setSymbol(PointSetItem::VOXEL, N_("Voxel"));

    itemSelectionChangedConnection.reset(
        ItemTreeView::instance()->sigSelectionChanged().connect(
            boost::bind(&MultiPointSetItemImpl::onItemSelectionChanged, this, _1)));
    subTreeChangedConnection.reset(
        self->sigSubTreeChanged().connect(
            boost::bind(&MultiPointSetItemImpl::onSubTreeChanged, this)));
}    


MultiPointSetItem::~MultiPointSetItem()
{
    delete impl;
}


SgNode* MultiPointSetItem::getScene()
{
    return impl->scene;
}


void MultiPointSetItemImpl::onItemSelectionChanged(ItemList<PointSetItem> items)
{
    bool changed = false;

    selectedPointSetItems.clear();
    for(size_t i=0; i < items.size(); ++i){
        PointSetItem* item = items[i];
        if(item->isOwnedBy(self)){
            selectedPointSetItems.push_back(item);
        }
    }
    
    if(!selectedPointSetItems.empty()){
        if(selectedPointSetItems != visiblePointSetItems){
            visiblePointSetItems = selectedPointSetItems;
            changed = true;
        }
    } else {
        ItemList<PointSetItem>::iterator p = visiblePointSetItems.begin();
        while(p != visiblePointSetItems.end()){
            if((*p)->isOwnedBy(self)){
                ++p;
            } else {
                p = visiblePointSetItems.erase(p);
                changed = true;
            }
        }
    }

    if(changed){
        scene->pointSetGroup->clearChildren();
        for(size_t i=0; i < visiblePointSetItems.size(); ++i){
            scene->pointSetGroup->addChild(visiblePointSetItems[i]->getScene());
        }
        scene->notifyUpdate(SgUpdate::ADDED | SgUpdate::REMOVED);
    }
}


void MultiPointSetItemImpl::onSubTreeChanged()
{
    pointSetItems.extractChildItems(self);

    ItemInfoMap prevMap(itemInfoMap);
    itemInfoMap.clear();

    for(size_t i=0; i < pointSetItems.size(); ++i){
        PointSetItem* item = pointSetItems[i];
        ItemInfoMap::iterator p = prevMap.find(item);
        if(p != prevMap.end()){
            p->second->index = i;
            itemInfoMap.insert(*p);
        } else {
            item->setPointSize(pointSize);
            item->setVoxelSize(voxelSize);
            item->setRenderingMode(renderingMode.which());
            
            PointSetItemInfo* info = new PointSetItemInfo;
            info->index = i;
            info->pointSetUpdateConnection.reset(
                item->pointSet()->sigUpdated().connect(
                    boost::bind(&MultiPointSetItemImpl::onPointSetUpdated, this, item)));
            itemInfoMap.insert(ItemInfoMap::value_type(item, info));

            sigPointSetItemAdded(i);
        }
    }
}


void MultiPointSetItemImpl::onPointSetUpdated(PointSetItem* item)
{
    ItemInfoMap::iterator p = itemInfoMap.find(item);
    if(p != itemInfoMap.end()){
        int index = p->second->index;
        sigPointSetItemUpdated(index);
    }
}


void MultiPointSetItem::setRenderingMode(int mode)
{
    impl->setRenderingMode(mode);
}


void MultiPointSetItemImpl::setRenderingMode(int mode)
{
    renderingMode.select(mode);
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setRenderingMode(mode);
    }
}


int MultiPointSetItem::renderingMode() const
{
    return impl->renderingMode.which();
}


double MultiPointSetItem::pointSize() const
{
    return impl->pointSize;
}
    

void MultiPointSetItem::setPointSize(double size)
{
    impl->setPointSize(size);
}


void MultiPointSetItemImpl::setPointSize(double size)
{
    pointSize = size;
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setPointSize(size);
    }
}


double MultiPointSetItem::voxelSize() const
{
    return impl->voxelSize;
}
    

void MultiPointSetItem::setVoxelSize(double size)
{
    impl->setVoxelSize(size);
}


void MultiPointSetItemImpl::setVoxelSize(double size)
{
    voxelSize = size;
    for(size_t i=0; i < pointSetItems.size(); ++i){
        pointSetItems[i]->setVoxelSize(size);
    }
}


int MultiPointSetItem::numPointSetItems() const
{
    return impl->pointSetItems.size();
}


PointSetItem* MultiPointSetItem::pointSetItem(int index)
{
    return impl->pointSetItems[index];
}


void MultiPointSetItem::selectSinglePointSetItem(int index)
{
    impl->selectSinglePointSetItem(index);
}


void MultiPointSetItemImpl::selectSinglePointSetItem(int index)
{
    if(index < 0 || index >= pointSetItems.size()){
        return;
    }
    
    itemSelectionChangedConnection.block();
    ItemTreeView* view = ItemTreeView::instance();
    for(size_t i=0; i < selectedPointSetItems.size(); ++i){
        view->unselectItem(selectedPointSetItems[i]);
    }
    PointSetItem* item = pointSetItems[index];
    view->selectItem(item);
    itemSelectionChangedConnection.unblock();

    ItemList<PointSetItem> items;
    items.push_back(item);
    onItemSelectionChanged(items);
}


SignalProxy<void(int index)> MultiPointSetItem::sigPointSetItemAdded()
{
    return impl->sigPointSetItemAdded;
}


SignalProxy<void(int index)> MultiPointSetItem::sigPointSetUpdated()
{
    return impl->sigPointSetItemUpdated;
}


int MultiPointSetItem::numAttentionPoints() const
{
    return impl->scene->numAttentionPoints();
}
        

Vector3 MultiPointSetItem::attentionPoint(int index) const
{
    return impl->scene->attentionPoint(index);
}


void MultiPointSetItem::clearAttentionPoints()
{
    return impl->scene->clearAttentionPoints(false);
}


void MultiPointSetItem::addAttentionPoint(const Vector3& p)
{
    impl->scene->addAttentionPoint(p, false);
}


SignalProxy<void()> MultiPointSetItem::sigAttentionPointsChanged()
{
    return impl->scene->sigAttentionPointsChanged;
}


void MultiPointSetItem::notifyAttentionPointChange()
{
    impl->scene->notifyAttentionPointChange();
}
    

ItemPtr MultiPointSetItem::doDuplicate() const
{
    return new MultiPointSetItem(*this);
}


void MultiPointSetItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Auto save"), false);
    putProperty(_("Directory"), "");
    putProperty(_("Rendering mode"), impl->renderingMode,
                boost::bind(&MultiPointSetItemImpl::onRenderingModePropertyChanged, impl, _1));
    putProperty.decimals(1).min(0.0)(_("Point size"), pointSize(),
                                     boost::bind(&MultiPointSetItem::setPointSize, this, _1), true);
    putProperty.decimals(4)(_("Voxel size"), voxelSize(),
                            boost::bind(&MultiPointSetItem::setVoxelSize, this, _1), true);
}


bool MultiPointSetItemImpl::onRenderingModePropertyChanged(int mode)
{
    if(mode != renderingMode.which()){
        if(renderingMode.select(mode)){
            setRenderingMode(mode);
            return true;
        }
    }
    return false;
}


bool MultiPointSetItem::store(Archive& archive)
{
    archive.write("autoSave", false);
    if(!impl->directory.empty()){
        archive.writeRelocatablePath("directory", impl->directory);
    }
    archive.write("renderingMode", impl->renderingMode.selectedSymbol());
    archive.write("pointSize", pointSize());
    archive.write("voxelSize", voxelSize());
    return true;
}


bool MultiPointSetItem::restore(const Archive& archive)
{
    archive.get("autoSave", false);
    string directory;
    if(archive.readRelocatablePath("directory", directory)){

    }
    string symbol;
    if(archive.read("renderingMode", symbol)){
        impl->onRenderingModePropertyChanged(impl->renderingMode.index(symbol));
    }
    setPointSize(archive.get("pointSize", pointSize()));
    setVoxelSize(archive.get("voxelSize", voxelSize()));
    
    return true;
}


SceneMultiPointSet::SceneMultiPointSet(MultiPointSetItemImpl* multiPointSetItem)
    : weakMultiPointSetItem(multiPointSetItem->self)
{
    pointSetGroup = new SgGroup;
    addChild(pointSetGroup);

    attentionPointMarkerGroup = new SgGroup;
    addChild(attentionPointMarkerGroup);
    
    isEditable = true;
}


int SceneMultiPointSet::numAttentionPoints() const
{
    return attentionPointMarkerGroup->numChildren();
}


Vector3 SceneMultiPointSet::attentionPoint(int index) const
{
    if(index < numAttentionPoints()){
        CrossMarker* marker = dynamic_cast<CrossMarker*>(attentionPointMarkerGroup->child(index));
        if(marker){
            return T() * marker->translation();
        }
    }
    return Vector3::Zero();
}


void SceneMultiPointSet::clearAttentionPoints(bool doNotify)
{
    if(!attentionPointMarkerGroup->empty()){
        attentionPointMarkerGroup->clearChildren();
        if(doNotify){
            notifyAttentionPointChange();
        }
    }
}


void SceneMultiPointSet::addAttentionPoint(const Vector3& point, bool doNotify)
{
    Vector3f color(1.0f, 1.0f, 0.0f);
    CrossMarker* marker = new CrossMarker(0.02, color);
    marker->setTranslation(T().inverse() * point);
    attentionPointMarkerGroup->addChild(marker);

    if(doNotify){
        notifyAttentionPointChange();
    }
}


void SceneMultiPointSet::setAttentionPoint(const Vector3& p, bool doNotify)
{
    clearAttentionPoints(false);
    addAttentionPoint(p, doNotify);
}


bool SceneMultiPointSet::removeAttentionPoint(const Vector3& point, double distanceThresh, bool doNotify)
{
    bool removed = false;

    SgGroup::iterator iter = attentionPointMarkerGroup->begin();
    while(iter != attentionPointMarkerGroup->end()){
        CrossMarker* marker = dynamic_cast<CrossMarker*>(iter->get());
        if(point.isApprox(marker->translation(), distanceThresh)){
            iter = attentionPointMarkerGroup->erase(iter);
            removed = true;
        } else {
            ++iter;
        }
    }
    if(removed && doNotify){
        notifyAttentionPointChange();
    }
    return removed;
}


void SceneMultiPointSet::notifyAttentionPointChange()
{
    attentionPointMarkerGroup->notifyUpdate();
    sigAttentionPointsChanged();
}

    
bool SceneMultiPointSet::onButtonPressEvent(const SceneWidgetEvent& event)
{
	if(!isEditable){
        return false;
    }
    
    bool processed = false;
    
    if(event.button() == Qt::LeftButton){
        if(event.modifiers() & Qt::ControlModifier){
            if(!removeAttentionPoint(event.point(), 0.01, true)){
                addAttentionPoint(event.point(), true);
            }
        } else {
            setAttentionPoint(event.point(), true);
        }
        processed = true;
    }
    
    return processed;

}


bool SceneMultiPointSet::onPointerMoveEvent(const SceneWidgetEvent& event)
{
    return false;
}


void SceneMultiPointSet::onContextMenuRequest(const SceneWidgetEvent& event, MenuManager& menuManager)
{
    menuManager.addItem(_("PointSet: Clear Attention Points"))->sigTriggered().connect(
        boost::bind(&SceneMultiPointSet::clearAttentionPoints, this, true));

    /*
    PointSetEraser* eraser = dynamic_cast<PointSetEraser*>(event.sceneWidget()->activeEventFilter());
    if(!eraser || eraser->weakPointSetItem.lock() != this->weakPointSetItem.lock()){
        menuManager.addItem(_("PointSet: Start Eraser Mode"))->sigTriggered().connect(
            boost::bind(&ScenePointSet::startEraserMode, this, event.sceneWidget()));
    }
    */
}


void SceneMultiPointSet::onSceneModeChanged(const SceneWidgetEvent& event)
{

}
