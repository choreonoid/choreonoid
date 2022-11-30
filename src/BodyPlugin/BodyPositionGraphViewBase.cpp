#include "BodyPositionGraphViewBase.h"
#include "BodySelectionManager.h"
#include <cnoid/RootItem>
#include <cnoid/Archive>
#include <cnoid/Link>

using namespace std;
using namespace cnoid;


BodyPositionGraphViewBase::BodyPositionGraphViewBase()
    : graph(this)
{
    setDefaultLayoutArea(BottomCenterArea);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    
    rootItemConnection = 
        RootItem::instance()->sigSelectedItemsChanged().connect(
            [&](const ItemList<>& selectedItems){
                onSelectedItemsChanged(selectedItems); });
}


QWidget* BodyPositionGraphViewBase::indicatorOnInfoBar()
{
    return &graph.statusLabel();
}


void BodyPositionGraphViewBase::onSelectedItemsChanged(ItemList<BodyMotionItem> items)
{
    if(items.empty()){
        return;
    }

    if(itemInfos.size() == items.size()){
        bool unchanged = true;
        int i=0;
        for(auto& info : itemInfos){
            if(info.item != items[i++]){
                unchanged = false;
                break;
            }
        }
        if(unchanged){
            return;
        }
    }
            
    itemInfos.clear();

    for(auto& item : items){
        if(auto bodyItem = item->findOwnerItem<BodyItem>()){
            itemInfos.emplace_back();
            auto& info = itemInfos.back();
            info.item = item;
            info.seq = item->motion()->positionSeq();
            info.bodyItem = bodyItem;

            info.connections.add(
                info.item->sigUpdated().connect(
                    [this, &info]{ onDataItemUpdated(info); }));

            auto it = itemInfos.end();
            --it;
            info.connections.add(
                info.item->sigDisconnectedFromRoot().connect(
                    [this, it](){ onDataItemDisconnectedFromRoot(it); }));
        }
    }

    updateBodyItems();
    setupGraphWidget();
}


void BodyPositionGraphViewBase::onDataItemDisconnectedFromRoot(std::list<ItemInfo>::iterator it)
{
    itemInfos.erase(it);
    updateBodyItems();
    setupGraphWidget();
}


void BodyPositionGraphViewBase::updateBodyItems()
{
    bodyItemConnections.disconnect();
    bodyItems.clear();
    
    for(auto& info : itemInfos){
        BodyItem* bodyItem = info.bodyItem;
        auto inserted = bodyItems.insert(bodyItem);
        if(inserted.second){
            bodyItemConnections.add(
                BodySelectionManager::instance()->sigLinkSelectionChanged(bodyItem).connect(
                    [this](const std::vector<bool>&){ setupGraphWidget(); }));
            bodyItemConnections.add(
                bodyItem->sigDisconnectedFromRoot().connect(
                    [this, bodyItem](){ onBodyItemDisconnectedFromRoot(bodyItem); }));
        }
    }
}


void BodyPositionGraphViewBase::onBodyItemDisconnectedFromRoot(BodyItem* bodyItem)
{
    bool erased = false;
    auto it = itemInfos.begin();
    while(it != itemInfos.end()){
        if(it->bodyItem == bodyItem){
            it = itemInfos.erase(it);
            erased = true;
        } else {
            ++it;
        }
    }
    if(erased){
        updateBodyItems();
        setupGraphWidget();
    }
}


void BodyPositionGraphViewBase::setupGraphWidget()
{
    auto bsm = BodySelectionManager::instance();
    graph.clearDataHandlers();

    for(auto& info : itemInfos){
        if(info.bodyItem){
            auto& seq = info.seq;
            auto body = info.bodyItem->body();
            auto linkSelection = bsm->linkSelection(info.bodyItem);
            for(size_t i=0; i < linkSelection.size(); ++i){
                if(linkSelection[i]){
                    if(auto link = body->link(i)){
                        addTrajectory(info, link, seq);
                    }
                }
            }
        }
    }
}


void BodyPositionGraphViewBase::onDataItemUpdated(ItemInfo& info)
{
    auto seq = info.seq;
    int newNumFrames = seq->numFrames();
    double newFrameRate = seq->frameRate();

    for(auto& handler : info.handlers){
        handler->setFrameProperties(newNumFrames, newFrameRate);
        handler->update();
    }
}


bool BodyPositionGraphViewBase::storeState(Archive& archive)
{
    return graph.storeState(archive);
}


bool BodyPositionGraphViewBase::restoreState(const Archive& archive)
{
    return graph.restoreState(archive);
}
