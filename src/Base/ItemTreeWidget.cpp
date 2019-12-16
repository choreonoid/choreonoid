#include "ItemTreeWidget.h"
#include "TreeWidget.h"
#include "PolymorphicItemFunctionSet.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "ItemManager.h"
#include "MenuManager.h"
#include "Archive.h"
#include <cnoid/ConnectionSet>
#include <QMouseEvent>
#include <QHeaderView>
#include <QModelIndex>
#include <QBoxLayout>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ItwItem : public QTreeWidgetItem
{
public:
    Item* item;
    ItemTreeWidget::Impl* widgetImpl;
    ScopedConnection itemSelectionConnection;
    ScopedConnection itemCheckConnection;
    bool isExpandedBeforeRemoving;

    ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl);
    virtual ~ItwItem();
    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;
};

}

namespace cnoid {

class ItemTreeWidget::Impl : public TreeWidget
{
public:
    ItemTreeWidget* self;
    RootItemPtr rootItem;
    ScopedConnection treeWidgetSelectionChangeConnection;
    ScopedConnectionSet rootItemConnections;
    vector<ItemPtr> topLevelItems;
    unordered_map<Item*, ItwItem*> itemToItwItemMap;
    ItemPtr lastClickedItem;
    map<int, int> checkIdToColumnMap;
    unordered_set<Item*> itemsBeingOperated;
    int isProcessingSlotForRootItemSignals;
    bool isCheckColumnShown;
    bool isDropping;

    function<bool(Item* item, bool isTopLevelItem)> isVisibleItem;

    ProjectManager* projectManager;
    ScopedConnectionSet projectManagerConnections;
    stack<bool> projectLoadingWithItemExpansionInfoStack;
    
    ItemList<Item> copiedItems;
    MenuManager menuManager;
    int fontPointSizeDiff;

    PolymorphicItemFunctionSet contextMenuFunctions;

    Impl(ItemTreeWidget* self, RootItem* rootItem);
    ~Impl();
    void initialize();
    void registerTopLevelItem(Item* item);
    bool registerTopLevelItemIter(Item* item, Item* newTopLevelItem, vector<ItemPtr>::iterator& pos);
    Item* findTopLevelItemOf(Item* item);
    void setCheckColumnShown(int column, bool on);
    void updateTreeWidgetItems();
    ItwItem* findItwItem(Item* item);
    ItwItem* findOrCreateItwItem(Item* item);
    void addCheckColumn(int checkId);
    void updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column);
    void releaseCheckColumn(int checkId);
    bool isItemBeingOperated(Item* item);
    void onSubTreeAddedOrMoved(Item* item);
    void insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItem);
    ItwItem* findNextItwItem(Item* item, bool isTopLevelItem);
    ItwItem* findNextItwItemInSubTree(Item* item, bool doTraverse);
    void onSubTreeRemoved(Item* item);
    void onItemAssigned(Item* assigned, Item* srcItem);

    ItemList<> getSelectedItems() const;
    void selectAllItems();
    void clearSelection();
    void setSelectedItemsChecked(bool on);
    void toggleSelectedItemChecks();
    void forEachTopItems(
        const ItemList<>& items, std::function<void(Item*, unordered_set<Item*>& itemSet)> callback);
    void copySelectedItems();
    void copySelectedItemsInSubTree(Item* item, Item* duplicated, unordered_set<Item*>& itemSet);
    void copySelectedItemsWithSubTrees();
    void cutSelectedItems();
    void pasteItems();
    void onTreeWidgetRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end);
    void onTreeWidgetSelectionChanged();
    void updateItemSelectionIter(QTreeWidgetItem* twItem, unordered_set<Item*>& selectedItemSet);
    void setItwItemSelected(ItwItem* itwItem, bool on);
    void toggleItwItemCheck(ItwItem* itwItem, int checkId, bool on);
    void zoomFontSize(int pointSizeDiff);
    
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void dropEvent(QDropEvent* event) override;

    void storeExpandedItems(Archive& archive);
    void storeExpandedItemsIter(QTreeWidgetItem* parentTwItem, Archive& archive, Listing& expanded);
    void onProjectAboutToBeLoaded(int recursiveLevel);
    void onProjectLoaded(int recursiveLevel);
    void restoreState(const Archive& archive, const ListingPtr expanded);
    void restoreExpandedItems(const Archive& archive, const ListingPtr expanded);
};

}


ItwItem::ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl)
    : item(item),
      widgetImpl(widgetImpl)
{
    widgetImpl->itemToItwItemMap[item] = this;

    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDropEnabled;
    if(widgetImpl->isCheckColumnShown){
        flags |= Qt::ItemIsUserCheckable;
    }
    if(!item->isSubItem()){
        flags |= Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
    }
    setFlags(flags);

    setToolTip(0, QString());

    widgetImpl->setItwItemSelected(this, item->isSelected());

    itemSelectionConnection =
        item->sigSelectionChanged().connect(
            [this](bool on){
                this->widgetImpl->setItwItemSelected(this, on); });

    if(widgetImpl->isCheckColumnShown){
        auto rootItem = widgetImpl->rootItem;
        int numCheckColumns = rootItem->numCheckEntries();
        for(int i=0; i < numCheckColumns; ++i){
            setCheckState(i + 1, item->isChecked(i) ? Qt::Checked : Qt::Unchecked);
            setToolTip(i + 1, rootItem->checkEntryDescription(i).c_str());
        }
        itemCheckConnection = 
            item->sigAnyCheckToggled().connect(
                [this](int checkId, bool on){
                    this->widgetImpl->toggleItwItemCheck(this, checkId, on); });
    }

    isExpandedBeforeRemoving = false;
}


ItwItem::~ItwItem()
{
    widgetImpl->itemToItwItemMap.erase(item);

    if(widgetImpl->lastClickedItem == item){
        widgetImpl->lastClickedItem = nullptr;
    }
}


QVariant ItwItem::data(int column, int role) const
{
    if((role == Qt::DisplayRole || role == Qt::EditRole) && column == 0){
        return item->name().c_str();
    }
    return QTreeWidgetItem::data(column, role);
}


void ItwItem::setData(int column, int role, const QVariant& value)
{
    if(column == 0){
        QTreeWidgetItem::setData(column, role, value);

        if(role == Qt::DisplayRole || role == Qt::EditRole){
            if(value.type() == QVariant::String){
                if(!value.toString().isEmpty()){
                    item->setName(value.toString().toStdString());
                }
            }
        }
    } else if(column >= 1 && role == Qt::CheckStateRole && widgetImpl->isCheckColumnShown){
        bool checked = ((Qt::CheckState)value.toInt() == Qt::Checked);
        QTreeWidgetItem::setData(column, role, value);
        int checkId = column - 1;

        if(checked != item->isChecked(checkId)){
            itemCheckConnection.block();
            item->setChecked(checkId, checked);
            itemCheckConnection.unblock();
        }
    }
}


ItemTreeWidget::ItemTreeWidget(RootItem* rootItem, QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this, rootItem);
    auto box = new QVBoxLayout;
    box->setContentsMargins(0, 0, 0, 0);
    box->addWidget(impl);
    setLayout(box);
}


ItemTreeWidget::Impl::Impl(ItemTreeWidget* self, RootItem* rootItem)
    : self(self),
      rootItem(rootItem),
      projectManager(ProjectManager::instance())
{
    initialize();
}


void ItemTreeWidget::Impl::initialize()
{
    isProcessingSlotForRootItemSignals = 0;
    isCheckColumnShown = true;
    isDropping = false;
    
    setColumnCount(1);

    auto hdr = header();
    hdr->setSectionResizeMode(0, QHeaderView::Stretch);
    hdr->setMinimumSectionSize(0);

    addCheckColumn(Item::PrimaryCheck);
    hdr->setStretchLastSection(false);
    hdr->swapSections(0, 1);

    setWordWrap(true);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setIndentation(12);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setDragDropMode(QAbstractItemView::InternalMove);

    treeWidgetSelectionChangeConnection =
        sigItemSelectionChanged().connect(
            [&](){ onTreeWidgetSelectionChanged(); });

    rootItemConnections.add(
        rootItem->sigCheckEntryAdded().connect(
            [&](int checkId){ addCheckColumn(checkId); }));

    rootItemConnections.add(
        rootItem->sigCheckEntryReleased().connect(
            [&](int checkId){ releaseCheckColumn(checkId); }));

    rootItemConnections.add(
        rootItem->sigSubTreeAdded().connect(
            [&](Item* item){ onSubTreeAddedOrMoved(item); }));

    rootItemConnections.add(
        rootItem->sigSubTreeMoved().connect(
            [&](Item* item){ onSubTreeAddedOrMoved(item); }));

    rootItemConnections.add(
        rootItem->sigSubTreeRemoved().connect(
            [&](Item* item, bool){ onSubTreeRemoved(item); }));

    rootItemConnections.add(
        rootItem->sigItemAssigned().connect(
            [&](Item* assigned, Item* srcItem){ onItemAssigned(assigned, srcItem); }));

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsAboutToBeRemoved(parent, start, end); });
    
    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsInserted(parent, start, end);  });

    projectManagerConnections.add(
        projectManager->sigProjectAboutToBeLoaded().connect(
            [&](int recursiveLevel){ onProjectAboutToBeLoaded(recursiveLevel); }));

    projectManagerConnections.add(
        projectManager->sigProjectLoaded().connect(
            [&](int recursiveLevel){ onProjectLoaded(recursiveLevel); }));

    isVisibleItem = [&](Item*, bool){ return true; };
    
    fontPointSizeDiff = 0;
}


ItemTreeWidget::~ItemTreeWidget()
{
    delete impl;
}


ItemTreeWidget::Impl::~Impl()
{
    clear();
}


RootItem* ItemTreeWidget::rootItem()
{
    return impl->rootItem;
}


void ItemTreeWidget::Impl::registerTopLevelItem(Item* item)
{
    auto pos = topLevelItems.begin();
    registerTopLevelItemIter(rootItem, item, pos);
}


bool ItemTreeWidget::Impl::registerTopLevelItemIter(Item* item, Item* newTopLevelItem, vector<ItemPtr>::iterator& pos)
{
    if(item == newTopLevelItem){
        topLevelItems.insert(pos, newTopLevelItem);
        return true;
    }
    if(pos != topLevelItems.end() && item == *pos){
        ++pos;
    }
    for(auto child = item->childItem(); child; child = child->nextItem()){
        if(registerTopLevelItemIter(child, newTopLevelItem, pos)){
            return true;
        }
    }
    return false;
}


Item* ItemTreeWidget::Impl::findTopLevelItemOf(Item* item)
{
    while(item){
        if(findItwItem(item)){
            return item;
        }
        item = item->parentItem();
    }
    return nullptr;
}


void ItemTreeWidget::setDragDropEnabled(bool on)
{
    if(on){
        impl->setDragDropMode(QAbstractItemView::InternalMove);
    } else {
        impl->setDragDropMode(QAbstractItemView::NoDragDrop);
    }        
}


void ItemTreeWidget::setCheckColumnShown(bool on)
{
    impl->isCheckColumnShown = on;
    for(auto& kv : impl->checkIdToColumnMap){
        int column = kv.second;
        impl->setCheckColumnShown(column, on);
    }
    impl->updateTreeWidgetItems();
}


void ItemTreeWidget::Impl::setCheckColumnShown(int column, bool on)
{
    if(on){
        header()->showSection(column);
        header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);
    } else {
        header()->hideSection(column);
    }
}
            
        
void ItemTreeWidget::setVisibleItemPredicate(std::function<bool(Item* item, bool isTopLevelItem)> pred)
{
    impl->isVisibleItem = pred;
}


void ItemTreeWidget::updateTreeWidgetItems()
{
    impl->updateTreeWidgetItems();
}


void ItemTreeWidget::Impl::updateTreeWidgetItems()
{
    clear();
    for(auto item = rootItem->childItem(); item; item = item->nextItem()){
        insertItem(invisibleRootItem(), item, true);
    }
}


void ItemTreeWidget::setExpanded(Item* item, bool on)
{
    if(auto itwItem = impl->findItwItem(item)){
        itwItem->setExpanded(on);
    }
}


ItwItem* ItemTreeWidget::Impl::findItwItem(Item* item)
{
    auto iter = itemToItwItemMap.find(item);
    if(iter != itemToItwItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


ItwItem* ItemTreeWidget::Impl::findOrCreateItwItem(Item* item)
{
    auto itwItem = findItwItem(item);
    if(!itwItem){
        itwItem = new ItwItem(item, this);
    }
    return itwItem;
}


void ItemTreeWidget::Impl::addCheckColumn(int checkId)
{
    if(checkIdToColumnMap.find(checkId) != checkIdToColumnMap.end()){
        return;
    }
    int column = columnCount();
    setColumnCount(column + 1);
    checkIdToColumnMap[checkId] = column;

    if(isCheckColumnShown){
        setCheckColumnShown(column, true);
        updateCheckColumnIter(invisibleRootItem(), checkId, column);
    }
}


void ItemTreeWidget::Impl::updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column)
{
    if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
        bool checked = itwItem->item->isChecked(checkId);
        itwItem->setCheckState(column, checked ? Qt::Checked : Qt::Unchecked);
    }
    const int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        updateCheckColumnIter(twItem->child(i), checkId, column);
    }
}


void ItemTreeWidget::Impl::releaseCheckColumn(int checkId)
{
    auto it = checkIdToColumnMap.find(checkId);
    if(it == checkIdToColumnMap.end()){
        return;
    }

    int releasedColumn = it->second;
    int numColumns = columnCount();
    checkIdToColumnMap.erase(it);
    
    if(releasedColumn < numColumns - 1){
        for(auto& kv : checkIdToColumnMap){
            int id = kv.first;
            int column = kv.second;
            if(column > releasedColumn){
                int reassignedColumn = column - 1;
                updateCheckColumnIter(invisibleRootItem(), checkId, reassignedColumn);
                kv.second = reassignedColumn;
            }
        }
    }

    header()->hideSection(releasedColumn);
    setColumnCount(numColumns - 1);
}


bool ItemTreeWidget::Impl::isItemBeingOperated(Item* item)
{
    return itemsBeingOperated.find(item) != itemsBeingOperated.end();
}


void ItemTreeWidget::Impl::onSubTreeAddedOrMoved(Item* item)
{
    if(isItemBeingOperated(item)){
        return;
    }
    
    auto parentItem = item->parentItem();
    if(auto parentItwItem = findItwItem(parentItem)){
        insertItem(parentItwItem, item, false);
    } else {
        bool isUpperLevelItemInserted = false;
        parentItem = parentItem->parentItem();
        while(parentItem){
            if(findItwItem(parentItem)){
                isUpperLevelItemInserted = true;
                break;
            }
            parentItem = parentItem->parentItem();
        }
        if(!isUpperLevelItemInserted){
            insertItem(invisibleRootItem(), item, true);
        }
    }
}


void ItemTreeWidget::Impl::insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItem)
{
    isProcessingSlotForRootItemSignals++;

    if(!isVisibleItem(item, isTopLevelItem)){
        if(!isTopLevelItem){
            parentTwItem = nullptr;
        }
    } else {
        auto itwItem = findOrCreateItwItem(item);
        if(isTopLevelItem){
            registerTopLevelItem(item);
        }
        auto nextItwItem = findNextItwItem(item, isTopLevelItem);
        if(nextItwItem){
            int index = parentTwItem->indexOfChild(nextItwItem);
            parentTwItem->insertChild(index, itwItem);
        } else {
            parentTwItem->addChild(itwItem);
        }

        if(projectLoadingWithItemExpansionInfoStack.empty() ||
           !projectLoadingWithItemExpansionInfoStack.top()){
            if(!parentTwItem->isExpanded() && !item->isSubItem()){
                parentTwItem->setExpanded(true);
            }
        }

        isTopLevelItem = false;
        parentTwItem = itwItem;
    }

    if(parentTwItem){
        for(Item* child = item->childItem(); child; child = child->nextItem()){
            insertItem(parentTwItem, child, isTopLevelItem);
        }
    }

    isProcessingSlotForRootItemSignals--;
}


ItwItem* ItemTreeWidget::Impl::findNextItwItem(Item* item, bool isTopLevelItem)
{
    auto nextItwItem = findNextItwItemInSubTree(item, isTopLevelItem);

    if(!nextItwItem){
        if(isTopLevelItem){
            auto it = ++std::find(topLevelItems.begin(), topLevelItems.end(), item);
            if(it != topLevelItems.end()){
                auto nextTopLevelItem = *it;
                nextItwItem = findItwItem(nextTopLevelItem);
            }
        }
    }
    
    return nextItwItem;
}


ItwItem* ItemTreeWidget::Impl::findNextItwItemInSubTree(Item* item, bool doTraverse)
{
    ItwItem* found = nullptr;

    for(auto nextItem = item->nextItem(); nextItem; nextItem = nextItem->nextItem()){
        if(found = findItwItem(nextItem)){
            break;
        }
        if(doTraverse){
            if(auto childItem = nextItem->childItem()){
                if(found = findNextItwItemInSubTree(childItem, true)){
                    break;
                }
            }
        }
    }
    
    return found;
}


void ItemTreeWidget::Impl::onSubTreeRemoved(Item* item)
{
    if(isItemBeingOperated(item)){
        return;
    }
    
    isProcessingSlotForRootItemSignals++;

    if(auto itwItem = findItwItem(item)){
        if(auto parentTwItem = itwItem->parent()){
            parentTwItem->removeChild(itwItem);
        } else {
            takeTopLevelItem(indexOfTopLevelItem(itwItem));
            std::remove(topLevelItems.begin(), topLevelItems.end(), item);
        }
        delete itwItem;
    } else {
        for(auto child = item->childItem(); child; child = child->nextItem()){
            onSubTreeRemoved(child);
        }
    }

    isProcessingSlotForRootItemSignals--;
}


void ItemTreeWidget::Impl::onItemAssigned(Item* assigned, Item* srcItem)
{
    if(findItwItem(assigned)){
        assigned->setSelected(srcItem->isSelected());
        int numCheckColumns = rootItem->numCheckEntries();
        for(int i=0; i < numCheckColumns; ++i){
            assigned->setChecked(i, srcItem->isChecked(i));
        }
    }
}


ItemList<> ItemTreeWidget::selectedItems() const
{
    return impl->getSelectedItems();
}


ItemList<> ItemTreeWidget::Impl::getSelectedItems() const
{
    auto selectedTwItems = selectedItems();
    ItemList<> items;
    items.reserve(selectedTwItems.size());
    for(auto& twItem : selectedTwItems){
        if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
            items.push_back(itwItem->item);
        }
    }
    return items;
}


void ItemTreeWidget::selectAllItems()
{
    impl->selectAllItems();
}


void ItemTreeWidget::Impl::selectAllItems()
{
    for(auto& kv : itemToItwItemMap){
        auto& item = kv.first;
        item->setSelected(true);
    }
}


void ItemTreeWidget::clearSelection()
{
    impl->clearSelection();
}


void ItemTreeWidget::Impl::clearSelection()
{
    for(auto& item : getSelectedItems()){
        item->setSelected(false);
    }
}


void ItemTreeWidget::setSelectedItemsChecked(bool on)
{
    impl->setSelectedItemsChecked(on);
}


void ItemTreeWidget::Impl::setSelectedItemsChecked(bool on)
{
    for(auto& item : getSelectedItems()){
        item->setChecked(on);
    }
}


void ItemTreeWidget::toggleSelectedItemChecks()
{
    impl->toggleSelectedItemChecks();
}


void ItemTreeWidget::Impl::toggleSelectedItemChecks()
{
    for(auto& item : getSelectedItems()){
        item->setChecked(!item->isChecked());
    }
}


void ItemTreeWidget::Impl::forEachTopItems
(const ItemList<>& items, std::function<void(Item*, unordered_set<Item*>& itemSet)> callback)
{
    unordered_set<Item*> itemSet(items.size());
    for(auto& item : items){
        itemSet.insert(item);
    }
    for(auto& item : items){
        bool isChild = false;
        auto parentItem = item->parentItem();
        while(parentItem){
            if(itemSet.find(parentItem) != itemSet.end()){
                isChild = true;
                break;
            }
            parentItem = parentItem->parentItem();
        }
        if(!isChild){
            callback(item, itemSet);
        }
    }
}


void ItemTreeWidget::copySelectedItems()
{
    impl->copySelectedItems();
}



void ItemTreeWidget::Impl::copySelectedItems()
{
    copiedItems.clear();

    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>& itemSet){
            if(auto duplicated = item->duplicate()){
                copiedItems.push_back(duplicated);
                copySelectedItemsInSubTree(item, duplicated, itemSet);
            }
        });
}


void ItemTreeWidget::Impl::copySelectedItemsInSubTree
(Item* item, Item* duplicated, unordered_set<Item*>& itemSet)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        ItemPtr duplicatedChild;
        if(itemSet.find(child) != itemSet.end()){
            if(child->isSubItem()){
                duplicatedChild = duplicated->findChildItem(child->name());
            } else {
                duplicatedChild = child->duplicate();
                if(duplicatedChild){
                    duplicated->addChildItem(duplicatedChild);
                }
            }
        }
        if(duplicatedChild){
            copySelectedItemsInSubTree(child, duplicatedChild, itemSet);
        } else {
            copySelectedItemsInSubTree(child, duplicated, itemSet);
        }
    }
}


void ItemTreeWidget::copySelectedItemsWithSubTrees()
{
    impl->copySelectedItemsWithSubTrees();
}


void ItemTreeWidget::Impl::copySelectedItemsWithSubTrees()
{
    copiedItems.clear();

    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>&){
            if(auto duplicated = item->duplicateSubTree()){
                copiedItems.push_back(duplicated);
            }
        });
}


void ItemTreeWidget::cutSelectedItems()
{
    impl->cutSelectedItems();
}


void ItemTreeWidget::Impl::cutSelectedItems()
{
    copiedItems.clear();
    
    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>&){
            copiedItems.push_back(item);
            item->detachFromParentItem();
        });
}


void ItemTreeWidget::pasteItems()
{
    impl->pasteItems();
}


void ItemTreeWidget::Impl::pasteItems()
{
    if(topLevelItems.empty()){
        return;
    }
    
    auto selected = getSelectedItems();
    ItemPtr parentItem;
    if(selected.empty()){
        parentItem = topLevelItems.front();
    } else {
        parentItem = selected.back();
    }
    if(parentItem){
        auto it = copiedItems.begin();
        while(it != copiedItems.end()){
            auto& item = *it;
            if(auto duplicated = item->duplicateSubTree()){
                parentItem->addChildItem(duplicated, true);
                ++it;
            } else {
                parentItem->addChildItem(item, true);
                it = copiedItems.erase(it);
            }
        }
    }
}


void ItemTreeWidget::Impl::onTreeWidgetRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(isProcessingSlotForRootItemSignals){
        return;
    }

    auto parentTwItem = itemFromIndex(parent);
    if(!parentTwItem){
        parentTwItem = invisibleRootItem();
    }

    vector<ItemPtr> items;
    for(int i=start; i <= end; ++i){
        if(auto itwItem = dynamic_cast<ItwItem*>(parentTwItem->child(i))){
            itwItem->isExpandedBeforeRemoving = itwItem->isExpanded();
            if(!isDropping){
                auto item = itwItem->item;
                if(!item->isSubItem()){
                    items.push_back(item);
                }
                itemsBeingOperated.insert(item);
            }
        }
    }

    for(auto item : items){
        item->detachFromParentItem();
    }

    itemsBeingOperated.clear();
}


void ItemTreeWidget::Impl::onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(isProcessingSlotForRootItemSignals){
        return;
    }
    
    auto parentTwItem = itemFromIndex(parent);
    if(parentTwItem){
        parentTwItem->setExpanded(true);
    } else {
        parentTwItem = invisibleRootItem();
    }

    auto parentItwItem = dynamic_cast<ItwItem*>(parentTwItem);
    auto parentItem = parentItwItem ? parentItwItem->item : rootItem.get();

    ItemPtr nextItem;
    if(end + 1 < parentTwItem->childCount()){
        if(auto nextItwItem = dynamic_cast<ItwItem*>(parentTwItem->child(end + 1))){
            nextItem = nextItwItem->item;
        }
    }
    
    vector<ItemPtr> items;
    for(int i=start; i <= end; ++i){
        if(auto itwItem = dynamic_cast<ItwItem*>(parentTwItem->child(i))){
            auto item = itwItem->item;
            if(!item->isSubItem()){
                items.push_back(item);
            }
            if(itwItem->isExpandedBeforeRemoving){
                itwItem->setExpanded(true);
            }
            itemsBeingOperated.insert(item);
        }
    }

    for(auto item : items){
        parentItem->insertChildItem(item, nextItem, true);
    }

    itemsBeingOperated.clear();    
}


void ItemTreeWidget::Impl::onTreeWidgetSelectionChanged()
{
    unordered_set<Item*> selectedItemSet;
    for(auto& twItem : selectedItems()){
        if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
            selectedItemSet.insert(itwItem->item);
        }
    }
    updateItemSelectionIter(invisibleRootItem(), selectedItemSet);
}


void ItemTreeWidget::Impl::updateItemSelectionIter(QTreeWidgetItem* twItem, unordered_set<Item*>& selectedItemSet)
{
    if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
        auto item = itwItem->item;
        bool on = selectedItemSet.find(item) != selectedItemSet.end();
        bool isFocused = (item == lastClickedItem);
        if(on != item->isSelected() || isFocused){
            itwItem->itemSelectionConnection.block();
            item->setSelected(on, isFocused);
            itwItem->itemSelectionConnection.unblock();
        }
    }
    int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        updateItemSelectionIter(twItem->child(i), selectedItemSet);
    }
}


void ItemTreeWidget::Impl::setItwItemSelected(ItwItem* itwItem, bool on)
{
    if(on != itwItem->isSelected()){
        treeWidgetSelectionChangeConnection.block();
        itwItem->setSelected(on);
        treeWidgetSelectionChangeConnection.unblock();
    }
}


void ItemTreeWidget::Impl::toggleItwItemCheck(ItwItem* itwItem, int checkId, bool on)
{
    if(on != itwItem->checkState(checkId + 1)){
        itwItem->setCheckState(checkId + 1, on ? Qt::Checked : Qt::Unchecked);
    }
}


void ItemTreeWidget::Impl::zoomFontSize(int pointSizeDiff)
{
    QFont font = TreeWidget::font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    setFont(font);
    fontPointSizeDiff += pointSizeDiff;
}


void ItemTreeWidget::Impl::mousePressEvent(QMouseEvent* event)
{
    ItwItem* itwItem = dynamic_cast<ItwItem*>(itemAt(event->pos()));
    Item* item = itwItem ? itwItem->item : rootItem.get();
    lastClickedItem = nullptr;

    // Emit sigSelectionChanged when clicking on an already selected item
    if(event->button() == Qt::LeftButton){
        auto selected = selectedItems();
        if(itwItem){
            if(selected.size() == 1 && itwItem == selected.front()){
                itwItem->itemSelectionConnection.block();
                itwItem->item->setSelected(true, true);
                itwItem->itemSelectionConnection.unblock();
            } else {
                lastClickedItem = itwItem->item;
            }
        }
    }
    
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        menuManager.setNewPopupMenu(this);
        contextMenuFunctions.dispatch(item);
        if(menuManager.numItems() > 0){
            menuManager.popupMenu()->popup(event->globalPos());
        }
    }
}


void ItemTreeWidget::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){

    case Qt::Key_Escape:
        clearSelection();
        break;
        
    case Qt::Key_Delete:
        cutSelectedItems();
        break;
        
    default:
        processed = false;
        break;
    }
        
    if(!processed && (event->modifiers() & Qt::ControlModifier)){

        processed = true;

        switch(event->key()){

        case Qt::Key_A:
            selectAllItems();
            break;
            
        case Qt::Key_R:
            ItemManager::reloadItems(getSelectedItems());
            break;
            
        case Qt::Key_Plus:
        case Qt::Key_Semicolon:
            zoomFontSize(1);
            break;
            
        case Qt::Key_Minus:
            zoomFontSize(-1);
            break;
            
        default:
            processed = false;
            break;
        }
    }

    if(!processed){
        TreeWidget::keyPressEvent(event);
    }
}


void ItemTreeWidget::Impl::dropEvent(QDropEvent* event)
{
    isDropping = true;
    TreeWidget::dropEvent(event);
    isDropping = false;
}


void ItemTreeWidget::setContextMenuFunctionFor
(const std::type_info& type,
 std::function<void(Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func)
{
    impl->contextMenuFunctions.setFunction(
        type,
        [this, func](Item* item){
            func(item, impl->menuManager, impl->contextMenuFunctions.dispatcher()); }
        );
}


bool ItemTreeWidget::storeState(Archive& archive)
{
    impl->storeExpandedItems(archive);
    return true;
}


void ItemTreeWidget::Impl::storeExpandedItems(Archive& archive)
{
    Listing& expanded = *archive.createFlowStyleListing("expanded");
    storeExpandedItemsIter(invisibleRootItem(), archive, expanded);
}


void ItemTreeWidget::Impl::storeExpandedItemsIter(QTreeWidgetItem* parentTwItem, Archive& archive, Listing& expanded)
{
    int n = parentTwItem->childCount();
    for(int i=0; i < n; ++i){
        if(auto itwItem = dynamic_cast<ItwItem*>(parentTwItem->child(i))){
            if(itwItem->isExpanded()){
                if(auto id = archive.getItemId(itwItem->item)){
                    expanded.append(id);
                }
            }
            if(itwItem->childCount() > 0){
                storeExpandedItemsIter(itwItem, archive, expanded);
            }
        }
    }
}


void ItemTreeWidget::Impl::onProjectAboutToBeLoaded(int recursiveLevel)
{
    projectLoadingWithItemExpansionInfoStack.push(false);
}


void ItemTreeWidget::Impl::onProjectLoaded(int recursiveLevel)
{
    if(!projectLoadingWithItemExpansionInfoStack.empty()){
        projectLoadingWithItemExpansionInfoStack.pop();
    }
}

bool ItemTreeWidget::restoreState(const Archive& archive)
{
    const ListingPtr expanded = archive.findListing("expanded");
    if(expanded->isValid()){
        if(!impl->projectLoadingWithItemExpansionInfoStack.empty()){
            impl->projectLoadingWithItemExpansionInfoStack.top() = true;
        }
    }
    archive.addPostProcess([&, expanded](){ impl->restoreState(archive, expanded); });
    return true;
}


void ItemTreeWidget::Impl::restoreState(const Archive& archive, const ListingPtr expanded)
{
    if(expanded->isValid()){
        restoreExpandedItems(archive, expanded);
    }
}


void ItemTreeWidget::Impl::restoreExpandedItems(const Archive& archive, const ListingPtr expanded)
{
    for(int i=0; i < expanded->size(); ++i){
        if(auto item = archive.findItem(expanded->at(i))){
            if(auto itwItem = findItwItem(item)){
                itwItem->setExpanded(true);
            }
        }
    }
}
