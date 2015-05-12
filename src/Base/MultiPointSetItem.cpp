/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiPointSetItem.h"
#include <cnoid/ItemTreeView>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;


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
    SgGroupPtr scene;
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
    scene = new SgGroup;
    
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
        scene->clearChildren();
        for(size_t i=0; i < visiblePointSetItems.size(); ++i){
            scene->addChild(visiblePointSetItems[i]->getScene());
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
