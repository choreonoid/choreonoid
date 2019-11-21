#include "ItemTreeWidget.h"
#include "TreeWidget.h"
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
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class ItwItem : public QTreeWidgetItem
{
public:
    Item* item;
    ItemTreeWidget::Impl* widgetImpl;
    ScopedConnectionSet itemConnections;
    bool isExpandedBeforeRemoving;

    ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl);
    virtual ~ItwItem();
    virtual QVariant data(int column, int role) const override;
    virtual void setData(int column, int role, const QVariant& value) override;
    void blockConnections();
    void unblockConnections();
};

}

namespace cnoid {

class ItemTreeWidget::Impl : public TreeWidget
{
public:
    ItemTreeWidget* self;
    RootItemPtr rootItem;
    ScopedConnectionSet rootItemConnections;
    vector<ItemPtr> topLevelItems;
    unordered_map<Item*, ItwItem*> itemToItwItemMap;
    map<int, int> checkIdToColumnMap;
    unordered_set<Item*> itemsBeingOperated;
    int isProcessingSlotForRootItemSignals;
    bool isDropping;
    ItemList<Item> copiedItems;
    ProjectManager* projectManager;
    Menu* popupMenu;
    MenuManager menuManager;

    Impl(ItemTreeWidget* self, RootItem* rootItem);
    void initialize();
    bool checkIfTopLevelItem(Item* item);
    Item* findTopLevelItemOf(Item* item);
    ItwItem* getItwItem(Item* item);
    ItwItem* getOrCreateItwItem(Item* item);
    void addCheckColumn(int checkId);
    void updateCheckColumn(int checkId, int column);
    void updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column);
    void releaseCheckColumn(int checkId);
    bool isItemBeingOperated(Item* item);
    void onSubTreeAddedOrMoved(Item* item);
    void insertItem(QTreeWidgetItem* parentTwItem, Item* item, Item* nextItem);
    void onSubTreeRemoved(Item* item, bool isMoving);
    //void onItemAssigned(Item* assigned, Item* srcItem);

    ItemList<> getSelectedItems();
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
    void setItwItemSelected(ItwItem* item, bool on);
    void toggleItwItemCheck(ItwItem* item, int checkId, bool on);
    
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void dropEvent(QDropEvent* event) override;
};

}


ItwItem::ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl)
    : item(item),
      widgetImpl(widgetImpl)
{
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsDropEnabled);

    if(!item->isSubItem()){
        setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsDragEnabled);
    }

    setToolTip(0, QString());

    itemConnections.add(
        item->sigSelectionChanged().connect(
            [&](bool on){
                widgetImpl->setItwItemSelected(this, on); }));
    
    itemConnections.add(
        item->sigCheckToggled().connect(
            [&](int checkId, bool on){
                widgetImpl->toggleItwItemCheck(this, checkId, on); }));
                
    auto rootItem = widgetImpl->rootItem;
    int numCheckColumns = rootItem->numCheckEntries();
    for(int i=0; i < numCheckColumns; ++i){
        setCheckState(i + 1, Qt::Unchecked);
        setToolTip(i + 1, rootItem->checkEntryDescription(i).c_str());
    }

    widgetImpl->itemToItwItemMap[item] = this;

    isExpandedBeforeRemoving = false;
}


ItwItem::~ItwItem()
{
    widgetImpl->itemToItwItemMap.erase(item);
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
    } else if(column >= 1 && role == Qt::CheckStateRole){
        bool checked = ((Qt::CheckState)value.toInt() == Qt::Checked);
        QTreeWidgetItem::setData(column, role, value);
        int checkId = column - 1;
        item->setChecked(checkId, checked);
    }
}


void ItwItem::blockConnections()
{
    itemConnections.block();
}


void ItwItem::unblockConnections()
{
    itemConnections.unblock();
}


ItemTreeWidget::ItemTreeWidget(RootItem* rootItem)
{
    impl = new Impl(this, rootItem);
    auto box = new QVBoxLayout;
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


ItemTreeWidget::~ItemTreeWidget()
{
    delete impl;
}


void ItemTreeWidget::Impl::initialize()
{
    isProcessingSlotForRootItemSignals = 0;
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
            [&](Item* item, bool isMoving){ onSubTreeRemoved(item, isMoving); }));

    /*
    rootItemConnections.add(
        rootItem->sigItemAssigned().connect([&](Item* assigned, Item* srcItem){ onItemAssigned(assigned, srcItem); }));
    */

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsAboutToBeRemoved(parent, start, end); });
    
    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsInserted(parent, start, end);  });

    popupMenu = new Menu(this);
    menuManager.setTopMenu(popupMenu);

    menuManager.addItem(_("Cut"))
        ->sigTriggered().connect([&](){ cutSelectedItems(); });
    menuManager.addItem(_("Copy (single)"))
        ->sigTriggered().connect([&](){ copySelectedItems(); });
    menuManager.addItem(_("Copy (sub tree)"))
        ->sigTriggered().connect([&](){ copySelectedItemsWithSubTrees(); });
    menuManager.addItem(_("Paste"))
        ->sigTriggered().connect([&](){ pasteItems(); });

    menuManager.addSeparator();
    menuManager.addItem(_("Check"))
        ->sigTriggered().connect([&](){ setSelectedItemsChecked(true); });
    menuManager.addItem(_("Uncheck"))
        ->sigTriggered().connect([&](){ setSelectedItemsChecked(false); });
    menuManager.addItem(_("Toggle checks"))
        ->sigTriggered().connect([&](){ toggleSelectedItemChecks(); });

    menuManager.addSeparator();
    menuManager.addItem(_("Reload"))
        ->sigTriggered().connect([&](){ ItemManager::reloadItems(getSelectedItems()); });

    menuManager.addSeparator();
    menuManager.addItem(_("Select all"))
        ->sigTriggered().connect([=](){ selectAllItems(); });
    menuManager.addItem(_("Clear selection"))
        ->sigTriggered().connect([=](){ clearSelection(); });
}


bool ItemTreeWidget::Impl::checkIfTopLevelItem(Item* item)
{
    auto it = std::find(topLevelItems.begin(), topLevelItems.end(), item);
    return it != topLevelItems.end();
}


Item* ItemTreeWidget::Impl::findTopLevelItemOf(Item* item)
{
    while(item){
        if(checkIfTopLevelItem(item)){
            return item;
        }
        item = item->parentItem();
    }
    return nullptr;
}


ItwItem* ItemTreeWidget::Impl::getItwItem(Item* item)
{
    auto iter = itemToItwItemMap.find(item);
    if(iter != itemToItwItemMap.end()){
        return iter->second;
    }
    return nullptr;
}


ItwItem* ItemTreeWidget::Impl::getOrCreateItwItem(Item* item)
{
    auto itwItem = getItwItem(item);
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
    header()->setSectionResizeMode(column, QHeaderView::ResizeToContents);

    updateCheckColumn(checkId, column);
}


void ItemTreeWidget::Impl::updateCheckColumn(int checkId, int column)
{
    // block signals
    updateCheckColumnIter(invisibleRootItem(), checkId, column);
    // unblock signals
}


void ItemTreeWidget::Impl::updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column)
{
    if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
        bool checked = itwItem->item->isChecked(checkId);
        itwItem->setCheckState(column, checked ? Qt::Checked : Qt::Unchecked);
        const int n = itwItem->childCount();
        for(int i=0; i < n; ++i){
            updateCheckColumnIter(itwItem->child(i), checkId, column);
        }
    }
}


void ItemTreeWidget::Impl::releaseCheckColumn(int checkId)
{
    auto it = checkIdToColumnMap.find(checkId);
    if(it != checkIdToColumnMap.end()){
        return;
    }

    int releasedColumn = it->second;
    int numColumns = columnCount();
    checkIdToColumnMap.erase(it);
    
    if(releasedColumn == numColumns - 1){
        setColumnCount(numColumns - 1);
        return;
    }

    for(auto& kv : checkIdToColumnMap){
        int id = kv.first;
        int column = kv.second;
        if(column > releasedColumn){
            int reassignedColumn = column - 1;
            updateCheckColumn(checkId, reassignedColumn);
            kv.second = reassignedColumn;
        }
    }
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
    
    isProcessingSlotForRootItemSignals++;

    if(auto topLevelItem = findTopLevelItemOf(item)){
        if(auto parentItem = item->parentItem()){
            if(parentItem == rootItem){
                insertItem(invisibleRootItem(), item, item->nextItem());
            } else {
                if(auto parentItwItem = getItwItem(parentItem)){
                    insertItem(parentItwItem, item, item->nextItem());
                }
            }
        }
        if(findTopLevelItemOf(topLevelItem)){
            std::remove(topLevelItems.begin(), topLevelItems.end(), topLevelItem);
        }
    }

    isProcessingSlotForRootItemSignals--;
}


void ItemTreeWidget::Impl::insertItem(QTreeWidgetItem* parentTwItem, Item* item, Item* nextItem)
{
    auto itwItem = getOrCreateItwItem(item);
    bool inserted = false;
    if(nextItem){
        if(auto nextItwItem = getItwItem(nextItem)){
            int index = parentTwItem->indexOfChild(nextItwItem);
            parentTwItem->insertChild(index, itwItem);
            inserted = true;
        }
    }
    if(!inserted){
        parentTwItem->addChild(itwItem);
    }

    if(!ProjectManager::instance()->isProjectBeingLoaded()){
        if(!parentTwItem->isExpanded() && !item->isSubItem()){
            parentTwItem->setExpanded(true);
        }
    }

    for(Item* child = item->childItem(); child; child = child->nextItem()){
        insertItem(itwItem, child, nullptr);
    }
}


void ItemTreeWidget::Impl::onSubTreeRemoved(Item* item, bool isMoving)
{
    if(isItemBeingOperated(item)){
        return;
    }
    
    isProcessingSlotForRootItemSignals++;
    
    if(auto itwItem = getItwItem(item)){
        if(auto parentTwItem = itwItem->parent()){
            parentTwItem->removeChild(itwItem);
        } else {
            takeTopLevelItem(indexOfTopLevelItem(itwItem));
            std::remove(topLevelItems.begin(), topLevelItems.end(), item);
        }
        delete itwItem;
    }

    isProcessingSlotForRootItemSignals--;
}


// What is this?
/*
void ItemTreeWidget::Impl::onItemAssigned(Item* assigned, Item* srcItem)
{

}
*/


ItemList<> ItemTreeWidget::Impl::getSelectedItems()
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


void ItemTreeWidget::Impl::selectAllItems()
{
    for(auto& kv : itemToItwItemMap){
        auto& item = kv.first;
        item->setSelected(true);
    }
}


void ItemTreeWidget::Impl::clearSelection()
{
    for(auto& item : getSelectedItems()){
        item->setSelected(false);
    }
}


void ItemTreeWidget::Impl::setSelectedItemsChecked(bool on)
{
    for(auto& item : getSelectedItems()){
        item->setChecked(on);
    }
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


void ItemTreeWidget::Impl::copySelectedItemsWithSubTrees()
{
    copiedItems.clear();

    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>&){
            if(auto duplicated = item->duplicateAll()){
                copiedItems.push_back(duplicated);
            }
        });
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
            if(auto duplicated = item->duplicateAll()){
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
    //! \todo unselect the items that is not selected in the tree widget
    for(auto& selected : selectedItems()){
        if(auto itwItem = dynamic_cast<ItwItem*>(selected)){
            itwItem->blockConnections();
            itwItem->item->setSelected(true);
            itwItem->unblockConnections();
        }
    }
}


void ItemTreeWidget::Impl::setItwItemSelected(ItwItem* item, bool on)
{

}


void ItemTreeWidget::Impl::toggleItwItemCheck(ItwItem* item, int checkId, bool on)
{

}


void ItemTreeWidget::Impl::mousePressEvent(QMouseEvent* event)
{
    // Emit sigSelectionChanged when clicking on an already selected item
    if(event->button() == Qt::LeftButton){
        auto selected = selectedItems();
        if(selected.size() == 1){
            if(auto itwItem = dynamic_cast<ItwItem*>(itemAt(event->pos()))){
                if(itwItem == selected.front()){
                    itwItem->blockConnections();
                    itwItem->item->setSelected(true, true);
                    itwItem->unblockConnections();
                }
            }
        }
    }
    
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        popupMenu->popup(event->globalPos());
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
            //zoomFontSize(1);
            break;
            
        case Qt::Key_Minus:
            //zoomFontSize(-1);
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
