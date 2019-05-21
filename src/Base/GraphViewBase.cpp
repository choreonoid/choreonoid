/**
   @author Shin'ichiro Nakaoka
*/

#include "GraphViewBase.h"
#include <cnoid/Item>
#include <cnoid/ConnectionSet>
#include <cnoid/ItemTreeView>
#include <cnoid/Archive>
#include <cnoid/TreeWidget>
#include <QBoxLayout>
#include <QHeaderView>
#include <QScrollBar>
#include <map>
#include <list>
#include <vector>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

namespace {
const bool TRACE_FUNCTIONS = false;

struct ItemInfo
{
    GraphViewBaseImpl* base;
    ItemPtr item;
    //ConnectionSet connections;
    std::vector<GraphDataHandlerPtr> handlers;

    ItemInfo(GraphViewBaseImpl* base) : base(base) { }
    ~ItemInfo();
};

class PartItem : public QTreeWidgetItem
{
public:
    int index;
};


class TreeWidgetEx : public TreeWidget
{
public:
    QSize modifiedSizeHint(QSize size) const {
        int width = sizeHintForColumn(0) * 3 / 2;
        width += verticalScrollBar()->sizeHint().width();
        size.setWidth(width);
        return size;
    }
    virtual QSize sizeHint() const {
        return modifiedSizeHint(QTreeWidget::sizeHint());
    }
    virtual QSize minimumSizeHint() const {
        return modifiedSizeHint(QTreeWidget::minimumSizeHint());
    }
};
    
}


namespace cnoid {

class GraphViewBaseImpl
{
public:

    GraphViewBaseImpl(GraphViewBase* self);
    ~GraphViewBaseImpl();

    GraphViewBase* self;
    QVBoxLayout* leftvbox;
    GraphWidget graph;
    TreeWidgetEx treeWidget;
    Connection itemSelectionChangedConnection;
    Connection partSelectionChangedConnection;

    list<ItemInfo> itemInfos;

    map<Item*, ConnectionSet> itemToConnectionSetMap;

    void onItemSelectionChanged(const ItemList<>& items);
    void onItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter);
    void updatePartList();
    void updateSelections();
    void onItemUpdated(std::list<ItemInfo>::iterator itemInfoIter);
    void notifyUpdateByEditing(Item* item);
};
}


namespace {

ItemInfo::~ItemInfo()
{
    map<Item*, ConnectionSet>::iterator p = base->itemToConnectionSetMap.find(item.get());
    if(p != base->itemToConnectionSetMap.end()){
        p->second.disconnect();
        base->itemToConnectionSetMap.erase(p);
    }
}
}
        

GraphViewBase::GraphViewBase()
{
    impl = new GraphViewBaseImpl(this);
}


GraphViewBaseImpl::GraphViewBaseImpl(GraphViewBase* self)
    : self(self),
      graph(self)
{
    treeWidget.setColumnCount(1);
    treeWidget.setHeaderHidden(true);
    treeWidget.setRootIsDecorated(false);
    treeWidget.setSelectionBehavior(QAbstractItemView::SelectRows);
    QHeaderView* header = treeWidget.header();
    header->setMinimumSectionSize(0);
    header->setStretchLastSection(false);
    header->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    treeWidget.setSelectionMode(QAbstractItemView::ExtendedSelection);
    treeWidget.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    treeWidget.setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    treeWidget.setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);

    leftvbox = new QVBoxLayout();
    leftvbox->setSpacing(0);
    leftvbox->addWidget(&treeWidget);
    
    QHBoxLayout* hbox = new QHBoxLayout();
    hbox->setSpacing(0);
    hbox->addLayout(leftvbox, 1);
    hbox->addWidget(&graph, 9999);
    self->setLayout(hbox);
    
    itemSelectionChangedConnection =
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&GraphViewBaseImpl::onItemSelectionChanged, this, _1));

    partSelectionChangedConnection =
        treeWidget.sigItemSelectionChanged().connect(
            std::bind(&GraphViewBaseImpl::updateSelections, this));
}


GraphViewBase::~GraphViewBase()
{
    delete impl;
}


GraphViewBaseImpl::~GraphViewBaseImpl()
{
    itemSelectionChangedConnection.disconnect();
    partSelectionChangedConnection.disconnect();
}


QVBoxLayout* GraphViewBase::leftVBox() const
{
    return impl->leftvbox;
}


QWidget* GraphViewBase::indicatorOnInfoBar()
{
    return &impl->graph.statusLabel();
}


void GraphViewBaseImpl::onItemSelectionChanged(const ItemList<>& allItems)
{
    ItemList<> items = self->extractTargetItems(allItems);

    if(items.empty()){
        return;
    }

    if(itemInfos.size() == items.size()){
        bool unchanged = true;
        int i=0;
        for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){
            if(it->item != items[i++]){
                unchanged = false;
                break;
            }
        }
        if(unchanged){
            return;
        }
    }

    itemInfos.clear();

    for(size_t i=0; i < items.size(); ++i){

        itemInfos.push_back(ItemInfo(this));
        list<ItemInfo>::iterator it = --itemInfos.end();
        it->item = items[i];

        ConnectionSet& connections = itemToConnectionSetMap[it->item.get()];
        connections.add(it->item->sigUpdated().connect(
                            std::bind(&GraphViewBaseImpl::onItemUpdated, this, it)));

        connections.add(it->item->sigDetachedFromRoot().connect(
                            std::bind(&GraphViewBaseImpl::onItemDetachedFromRoot, this, it)));
    }

    updatePartList();
}


void GraphViewBaseImpl::onItemDetachedFromRoot(std::list<ItemInfo>::iterator itemInfoIter)
{
    itemInfos.erase(itemInfoIter);
    updatePartList();
}


int GraphViewBase::currentNumParts(const ItemList<>& items) const
{
    return 0;
}
    

void GraphViewBaseImpl::updatePartList()
{
    treeWidget.clear();

    int numParts = 0;
    if(!itemInfos.empty()){
        ItemList<Item> items;
        for(list<ItemInfo>::iterator p = itemInfos.begin(); p != itemInfos.end(); ++p){
            items.push_back(p->item.get());
        }
        numParts = self->currentNumParts(items);
    }
    
    for(int i=0; i < numParts; ++i){
        PartItem* partItem = new PartItem();
        partItem->setText(0, QString::number(i));
        partItem->setTextAlignment(0, Qt::AlignHCenter);
        partItem->index = i;
        treeWidget.addTopLevelItem(partItem);
    }

    treeWidget.header()->updateGeometry();
    treeWidget.updateGeometry();

    updateSelections();
}


void GraphViewBase::updateSelections()
{
    impl->updateSelections();
}


void GraphViewBaseImpl::updateSelections()
{
    if(TRACE_FUNCTIONS){
        cout << "GraphViewBaseImpl::updateSelections()" << endl;
    }
    
    graph.clearDataHandlers();
    vector<GraphDataHandlerPtr> tmpHandlers;

    for(list<ItemInfo>::iterator it = itemInfos.begin(); it != itemInfos.end(); ++it){
        Item* item = it->item.get();
        QList<QTreeWidgetItem*> selected = treeWidget.selectedItems();
        for(int i=0; i < selected.size(); ++i){
            PartItem* part = static_cast<PartItem*>(selected[i]);
            tmpHandlers.clear();
            self->addGraphDataHandlers(item, part->index, tmpHandlers);
            if(!tmpHandlers.empty()){
                for(size_t j=0; j < tmpHandlers.size(); ++j){
                    graph.addDataHandler(tmpHandlers[j]);
                    it->handlers.push_back(tmpHandlers[j]);
                }
            }
        }
    }
}


void GraphViewBaseImpl::onItemUpdated(std::list<ItemInfo>::iterator itemInfoIter)
{
    if(TRACE_FUNCTIONS){
        cout << "GraphViewBaseImpl::onItemUpdated()" << endl;
    }

    for(size_t i=0; i < itemInfoIter->handlers.size(); ++i){
        self->updateGraphDataHandler(itemInfoIter->item.get(), itemInfoIter->handlers[i]);
    }
}


void GraphViewBase::notifyUpdateByEditing(Item* item)
{
    impl->notifyUpdateByEditing(item);
}


void GraphViewBaseImpl::notifyUpdateByEditing(Item* item)
{
    map<Item*, ConnectionSet>::iterator p = itemToConnectionSetMap.find(item);
    if(p != itemToConnectionSetMap.end()){
        p->second.block();
        item->notifyUpdate();
        p->second.unblock();
    }
}


bool GraphViewBase::storeState(Archive& archive)
{
    return impl->graph.storeState(archive);
}


bool GraphViewBase::restoreState(const Archive& archive)
{
    return impl->graph.restoreState(archive);
}
