#ifndef CNOID_BODY_PLUGIN_BODY_POSITION_GRAPH_VIEW_BASE_H
#define CNOID_BODY_PLUGIN_BODY_POSITION_GRAPH_VIEW_BASE_H

#include "BodyItem.h"
#include "BodyMotionItem.h"
#include <cnoid/BodyPositionSeq>
#include <cnoid/View>
#include <cnoid/GraphWidget>
#include <cnoid/ItemList>
#include <cnoid/ConnectionSet>
#include <set>

namespace cnoid {

class BodyPositionGraphViewBase : public View
{
public:
    BodyPositionGraphViewBase();
            
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;
            
protected:
    virtual QWidget* indicatorOnInfoBar() override;
            
    GraphWidget graph;
    ScopedConnection rootItemConnection;

    struct ItemInfo
    {
        ~ItemInfo(){
            connections.disconnect();
        }
        BodyMotionItemPtr item;
        std::shared_ptr<BodyPositionSeq> seq;
        BodyItemPtr bodyItem;
        ScopedConnectionSet connections;
        std::vector<GraphDataHandlerPtr> handlers;
    };

    std::list<ItemInfo> itemInfos;

    std::set<BodyItemPtr> bodyItems;
    ScopedConnectionSet bodyItemConnections;

    void onSelectedItemsChanged(ItemList<BodyMotionItem> items);
    void onDataItemDisconnectedFromRoot(std::list<ItemInfo>::iterator it);
    void updateBodyItems();
    void onBodyItemDisconnectedFromRoot(BodyItem* bodyItem);
    void setupGraphWidget();
    void onDataItemUpdated(ItemInfo& info);

    virtual void addTrajectory(ItemInfo& info, Link* link, std::shared_ptr<BodyPositionSeq> seq) = 0;
};

}

#endif
