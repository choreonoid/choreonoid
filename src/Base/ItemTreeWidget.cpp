#include "ItemTreeWidget.h"
#include "TreeWidget.h"
#include "PolymorphicItemFunctionSet.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "ItemManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "Archive.h"
#include <cnoid/ConnectionSet>
#include <QMouseEvent>
#include <QHeaderView>
#include <QModelIndex>
#include <QBoxLayout>
//#include <QMimeData>
#include <fmt/format.h>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

class ItwItem : public QTreeWidgetItem
{
public:
    Item* item;
    ItemTreeWidget::Impl* widgetImpl;
    ScopedConnection itemSelectionConnection;
    ScopedConnection itemCheckConnection;
    ScopedConnection displayUpdateConnection;
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
    RootItemPtr projectRootItem;
    ScopedConnectionSet projectRootItemConnections;
    ConnectionSet subTreeAddedOrMovedConnections;
    ItemPtr localRootItem;
    ScopedConnection localRootItemConnection;
    bool isProcessingSlotOnlocalRootItemPositionChanged;
    std::function<Item*(bool doCreate)> localRootItemUpdateFunction;
    ScopedConnection treeWidgetSelectionChangeConnection;
    vector<ItemPtr> topLevelItems;
    unordered_map<Item*, ItwItem*> itemToItwItemMap;
    ItemPtr lastClickedItem;
    map<int, int> checkIdToColumnMap;
    unordered_set<Item*> itemsUnderTreeWidgetInternalOperation;

    // The use of this variable might be replaced with just blocking
    // the connections to TreeWidget's signals when calling TreeWidget's
    // functions to change its tree structure.
    int isChangingTreeWidgetTreeStructure;
    
    bool isCheckColumnShown;

    PolymorphicItemFunctionSet  visibilityFunctions;
    bool visibilityFunction_isTopLevelItemCandidate;
    bool visibilityFunction_result;

    PolymorphicItemFunctionSet  displayFunctions;
    Display itemDisplay;

    ItemList<Item> dragItems;
    bool isDropping;

    PolymorphicItemFunctionSet  positionAcceptanceFunctions;
    mutable bool positionAcceptanceFunction_result;
    mutable Item* positionAcceptanceFunction_parentItem;

    ProjectManager* projectManager;
    ScopedConnectionSet projectManagerConnections;
    stack<bool> projectLoadingWithItemExpansionInfoStack;
    
    ItemList<Item> copiedItems;
    MenuManager menuManager;
    int fontPointSizeDiff;

    PolymorphicItemFunctionSet  contextMenuFunctions;

    Impl(ItemTreeWidget* self);
    ~Impl();
    void initialize();
    Item* findOrCreateLocalRootItem(bool doCreate);
    void setLocalRootItem(Item* item);
    void setCheckColumnShown(int column, bool on);
    bool checkPositionAcceptance(Item* item, Item* parentItem) const;
    void clearTreeWidgetItems();
    void updateTreeWidgetItems();
    ItwItem* findItwItem(Item* item);
    ItwItem* findOrCreateItwItem(Item* item);
    void addCheckColumn(int checkId);
    void updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column);
    void releaseCheckColumn(int checkId);
    void registerTopLevelItem(Item* item);
    bool registerTopLevelItemIter(Item* item, Item* newTopLevelItem, vector<ItemPtr>::iterator& pos);
    void unregisterTopLevelItem(Item* item);
    void updateItemDisplay(ItwItem* itwItem);
    void insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItemCandidate);
    ItwItem* findNextItwItem(Item* item, bool isTopLevelItem);
    ItwItem* findNextItwItemInSubTree(Item* item, bool doTraverse);
    bool isItemUnderTreeWidgetInternalOperation(Item* item);
    void onSubTreeAddedOrMoved(Item* item);
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
    bool pasteItems(bool doCheckPositionAcceptance);
    bool checkPastable(Item* pasteParentItem) const;
    void onTreeWidgetRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end);
    void onTreeWidgetSelectionChanged();
    void updateItemSelectionIter(QTreeWidgetItem* twItem, unordered_set<Item*>& selectedItemSet);
    void setItwItemSelected(ItwItem* itwItem, bool on);
    void toggleItwItemCheck(ItwItem* itwItem, int checkId, bool on);
    void zoomFontSize(int pointSizeDiff);
    
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void dragEnterEvent(QDragEnterEvent *event) override;
    virtual void dragMoveEvent(QDragMoveEvent *event) override;
    virtual void dragLeaveEvent(QDragLeaveEvent *event) override;
    virtual void dropEvent(QDropEvent* event) override;

    void storeExpandedItems(Archive& archive);
    void storeExpandedItemsIter(QTreeWidgetItem* parentTwItem, Archive& archive, Listing& expanded);
    void onProjectAboutToBeLoaded(int recursiveLevel);
    void onProjectLoaded(int recursiveLevel);
    void restoreState(const Archive& archive, const ListingPtr expanded);
    void restoreExpandedItems(const Archive& archive, const ListingPtr expanded);
};

}


QBrush ItemTreeWidget::Display::foreground() const
{
    return item->foreground(0);
}

void ItemTreeWidget::Display::setForeground(const QBrush& brush)
{
    item->setForeground(0, brush);
}

QBrush ItemTreeWidget::Display::background() const
{
    return item->background(0);
}
    
void ItemTreeWidget::Display::setBackground(const QBrush& brush)
{
    item->setBackground(0, brush);
}

QFont ItemTreeWidget::Display::font() const
{
    return item->font(0);
}

void ItemTreeWidget::Display::setFont(const QFont& font)
{
    item->setFont(0, font);
}

QIcon ItemTreeWidget::Display::icon() const
{
    return item->icon(0);
}

void ItemTreeWidget::Display::setIcon(const QIcon& icon)
{
    item->setIcon(0, icon);
}

void ItemTreeWidget::Display::setText(const std::string& text)
{
    item->setText(0, text.c_str());
}

void ItemTreeWidget::Display::setToolTip(const std::string& toolTip)
{
    item->setText(0, toolTip.c_str());
}

void ItemTreeWidget::Display::setStatusTip(const std::string& statusTip)
{
    item->setStatusTip(0, statusTip.c_str());
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
        auto rootItem = widgetImpl->projectRootItem;
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

    if(widgetImpl->displayFunctions.hasFunctionFor(item)){
        widgetImpl->updateItemDisplay(this);
        displayUpdateConnection =
            item->sigUpdated().connect(
                [this](){ this->widgetImpl->updateItemDisplay(this); });
    }
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


ItemTreeWidget::ItemTreeWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
    auto box = new QVBoxLayout;
    box->setContentsMargins(0, 0, 0, 0);
    box->addWidget(impl);
    setLayout(box);
}


ItemTreeWidget::Impl::Impl(ItemTreeWidget* self)
    : self(self),
      projectRootItem(RootItem::instance()),
      localRootItem(projectRootItem),
      projectManager(ProjectManager::instance())
{
    initialize();
}


ItemTreeWidget::~ItemTreeWidget()
{
    delete impl;
}


ItemTreeWidget::Impl::~Impl()
{
    clear();
}


void ItemTreeWidget::Impl::initialize()
{
    isChangingTreeWidgetTreeStructure = 0;
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

    projectRootItemConnections.add(
        projectRootItem->sigCheckEntryAdded().connect(
            [&](int checkId){ addCheckColumn(checkId); }));

    projectRootItemConnections.add(
        projectRootItem->sigCheckEntryReleased().connect(
            [&](int checkId){ releaseCheckColumn(checkId); }));

    subTreeAddedOrMovedConnections.add(
        projectRootItem->sigSubTreeAdded().connect(
            [&](Item* item){ onSubTreeAddedOrMoved(item); }));

    subTreeAddedOrMovedConnections.add(
        projectRootItem->sigSubTreeMoved().connect(
            [&](Item* item){ onSubTreeAddedOrMoved(item); }));

    projectRootItemConnections.add(subTreeAddedOrMovedConnections);

    projectRootItemConnections.add(
        projectRootItem->sigSubTreeRemoved().connect(
            [&](Item* item, bool){ onSubTreeRemoved(item); }));

    projectRootItemConnections.add(
        projectRootItem->sigItemAssigned().connect(
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

    isProcessingSlotOnlocalRootItemPositionChanged = false;
    
    fontPointSizeDiff = 0;
}


RootItem* ItemTreeWidget::projectRootItem()
{
    return impl->projectRootItem;
}


Item* ItemTreeWidget::findRootItem()
{
    return impl->findOrCreateLocalRootItem(false);
}


Item* ItemTreeWidget::findOrCreateRootItem()
{
    return impl->findOrCreateLocalRootItem(true);
}


Item* ItemTreeWidget::Impl::findOrCreateLocalRootItem(bool doCreate)
{
    if(!localRootItem){
        if(localRootItemUpdateFunction){
            if(auto item = localRootItemUpdateFunction(doCreate)){
                setLocalRootItem(item);
            }
        }
    }
    return localRootItem;
}


void ItemTreeWidget::setRootItem(Item* item)
{
    impl->setLocalRootItem(item);
}


void ItemTreeWidget::Impl::setLocalRootItem(Item* item)
{
    if(item != localRootItem){
        localRootItem = item;
        localRootItemConnection.disconnect();

        if(item && item != projectRootItem){
            // When the position of the local root item is changed,
            // the local root item may also be changed.
            // Check it in the following callback function.
            localRootItemConnection =
                item->sigPositionChanged2().connect(
                    [&,item](Item* topItem, Item* prevTopParentItem){
                        if(!isProcessingSlotOnlocalRootItemPositionChanged){
                            isProcessingSlotOnlocalRootItemPositionChanged = true;
                            if(prevTopParentItem){ // Exclude a newly added item
                                setLocalRootItem(nullptr);

                                // The following code is dangerous because it may add a
                                // connection to sigPositionChanged2 during processing the
                                // signal. It may go into an infinte loop to call this
                                // lambda function. It's better to redesign the logic.

                                // findOrCreateLocalRootItem(false);
                            }
                            isProcessingSlotOnlocalRootItemPositionChanged = false;
                        }
                    });
        }
        
        updateTreeWidgetItems();
    }
}


void ItemTreeWidget::setRootItemUpdateFunction(std::function<Item*(bool doCreate)> callback)
{
    impl->localRootItemUpdateFunction = callback;
    impl->localRootItem = nullptr;
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
            

void ItemTreeWidget::customizeVisibility_
(const std::type_info& type, std::function<bool(Item* item, bool isTopLevelItemCandidate)> func)
{
    impl->visibilityFunctions.setFunction(
        type,
        [this, func](Item* item){
            impl->visibilityFunction_result =
                func(item, impl->visibilityFunction_isTopLevelItemCandidate); });
}


void ItemTreeWidget::customizeDisplay_
(const std::type_info& type, std::function<void(Item* item, Display& display)> func)
{
    impl->displayFunctions.setFunction(
        type,
        [this, func](Item* item){ func(item, impl->itemDisplay); });
}


void ItemTreeWidget::customizePositionAcceptance_
(const std::type_info& type, std::function<bool(Item* item, Item* parentItem)> func)
{
    impl->positionAcceptanceFunctions.setFunction(
        type,
        [this, func](Item* item){
            impl->positionAcceptanceFunction_result =
                func(item, impl->positionAcceptanceFunction_parentItem); });
}


bool ItemTreeWidget::checkPositionAcceptance(Item* item, Item* parentItem) const
{
    return impl->checkPositionAcceptance(item, parentItem);
}


bool ItemTreeWidget::Impl::checkPositionAcceptance(Item* item, Item* parentItem) const
{
    positionAcceptanceFunction_parentItem = parentItem;
    positionAcceptanceFunction_result = true;
    positionAcceptanceFunctions.dispatch(item);
    return positionAcceptanceFunction_result;
}
    

void ItemTreeWidget::customizeContextMenu_
(const std::type_info& type,
 std::function<void(Item* item, MenuManager& menuManager, ItemFunctionDispatcher menuFunction)> func)
{
    impl->contextMenuFunctions.setFunction(
        type,
        [this, func](Item* item){
            func(item, impl->menuManager, impl->contextMenuFunctions.dispatcher()); }
        );
}


void ItemTreeWidget::updateTreeWidgetItems()
{
    impl->updateTreeWidgetItems();
}


void ItemTreeWidget::Impl::clearTreeWidgetItems()
{
    clear();
    topLevelItems.clear();
}


void ItemTreeWidget::Impl::updateTreeWidgetItems()
{
    clearTreeWidgetItems();

    if(localRootItem){
        isChangingTreeWidgetTreeStructure++;
        for(auto item = localRootItem->childItem(); item; item = item->nextItem()){
            insertItem(invisibleRootItem(), item, true);
        }
        isChangingTreeWidgetTreeStructure--;
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


void ItemTreeWidget::Impl::registerTopLevelItem(Item* item)
{
    auto pos = topLevelItems.begin();
    registerTopLevelItemIter(localRootItem, item, pos);
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


void ItemTreeWidget::Impl::unregisterTopLevelItem(Item* item)
{
    topLevelItems.erase(
        std::remove(topLevelItems.begin(), topLevelItems.end(), item),
        topLevelItems.end());
}
 

void ItemTreeWidget::Impl::updateItemDisplay(ItwItem* itwItem)
{
    itemDisplay.item = itwItem;
    displayFunctions.dispatch(itwItem->item);
}


void ItemTreeWidget::Impl::insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItemCandidate)
{
    if(!findOrCreateLocalRootItem(false)){
        return;
    }

    bool isVisible = item->isOwnedBy(localRootItem);
    if(isVisible){
        visibilityFunction_isTopLevelItemCandidate = isTopLevelItemCandidate;
        visibilityFunction_result = true;
        visibilityFunctions.dispatch(item);
        isVisible = visibilityFunction_result;
    }
    
    if(!isVisible){
        if(!isTopLevelItemCandidate){
            parentTwItem = nullptr;
        }
    } else {
        auto itwItem = findOrCreateItwItem(item);
        if(isTopLevelItemCandidate){
            registerTopLevelItem(item);
        }
        auto nextItwItem = findNextItwItem(item, isTopLevelItemCandidate);
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
        isTopLevelItemCandidate = false;
        parentTwItem = itwItem;
    }

    if(parentTwItem){
        for(Item* child = item->childItem(); child; child = child->nextItem()){
            insertItem(parentTwItem, child, isTopLevelItemCandidate);
        }
    }
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


bool ItemTreeWidget::Impl::isItemUnderTreeWidgetInternalOperation(Item* item)
{
    return itemsUnderTreeWidgetInternalOperation.find(item) != itemsUnderTreeWidgetInternalOperation.end();
}


void ItemTreeWidget::Impl::onSubTreeAddedOrMoved(Item* item)
{
    if(isItemUnderTreeWidgetInternalOperation(item)){
        return;
    }

    subTreeAddedOrMovedConnections.block();

    isChangingTreeWidgetTreeStructure++;

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

    isChangingTreeWidgetTreeStructure--;

    subTreeAddedOrMovedConnections.unblock();
}


void ItemTreeWidget::Impl::onSubTreeRemoved(Item* item)
{
    if(isItemUnderTreeWidgetInternalOperation(item) || !localRootItem){
        return;
    }
    
    isChangingTreeWidgetTreeStructure++;

    if(auto itwItem = findItwItem(item)){
        if(auto parentTwItem = itwItem->parent()){
            parentTwItem->removeChild(itwItem);
        } else {
            takeTopLevelItem(indexOfTopLevelItem(itwItem));
            unregisterTopLevelItem(item);
        }
        delete itwItem;
    } else {
        for(auto child = item->childItem(); child; child = child->nextItem()){
            onSubTreeRemoved(child);
        }
    }

    isChangingTreeWidgetTreeStructure--;
}


void ItemTreeWidget::Impl::onItemAssigned(Item* assigned, Item* srcItem)
{
    if(findItwItem(assigned)){
        assigned->setSelected(srcItem->isSelected());
        int numCheckColumns = projectRootItem->numCheckEntries();
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


bool ItemTreeWidget::pasteItems(bool doCheckPositionAcceptance)
{
    return impl->pasteItems(doCheckPositionAcceptance);
}


bool ItemTreeWidget::Impl::pasteItems(bool doCheckPositionAcceptance)
{
    bool pasted = false;
    auto selected = getSelectedItems();
    ItemPtr parentItem;
    if(selected.empty()){
        parentItem = localRootItem;
    } else {
        parentItem = selected.back();
    }
    if(parentItem && !copiedItems.empty()){
        bool isPastable = true;
        if(doCheckPositionAcceptance){
            if(!checkPastable(parentItem)){
                showWarningDialog(
                    format(_("The copied items cannot be pasted to \"{0}\"."),
                           parentItem->name()));
                isPastable = false;
            }
        }
        if(isPastable){
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
                pasted = true;
            }
        }
    }
    return pasted;
}


bool ItemTreeWidget::checkPastable(Item* pasteParentItem) const
{
    return impl->checkPastable(pasteParentItem);
}


bool ItemTreeWidget::Impl::checkPastable(Item* pasteParentItem) const
{
    for(auto& item : copiedItems){
        if(!checkPositionAcceptance(item, pasteParentItem)){
            return false;
        }
    }
    return true;
}


void ItemTreeWidget::Impl::onTreeWidgetRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(isChangingTreeWidgetTreeStructure){
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
                itemsUnderTreeWidgetInternalOperation.insert(item);
            }
        }
    }

    for(auto item : items){
        item->detachFromParentItem();
    }

    itemsUnderTreeWidgetInternalOperation.clear();
}


void ItemTreeWidget::Impl::onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(isChangingTreeWidgetTreeStructure || !localRootItem){
        return;
    }
    
    auto parentTwItem = itemFromIndex(parent);
    if(parentTwItem){
        parentTwItem->setExpanded(true);
    } else {
        parentTwItem = invisibleRootItem();
    }

    auto parentItwItem = dynamic_cast<ItwItem*>(parentTwItem);
    auto parentItem = parentItwItem ? parentItwItem->item : localRootItem.get();

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
            itemsUnderTreeWidgetInternalOperation.insert(item);
        }
    }

    for(auto item : items){
        parentItem->insertChildItem(item, nextItem, true);
    }

    itemsUnderTreeWidgetInternalOperation.clear();    
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
    Item* item = itwItem ? itwItem->item : findOrCreateLocalRootItem(true);
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

    if(item && event->button() == Qt::RightButton){
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


void ItemTreeWidget::Impl::dragEnterEvent(QDragEnterEvent* event)
{
    auto srcWidget = dynamic_cast<ItemTreeWidget::Impl*>(event->source());
    if(srcWidget && srcWidget != this){ // drag from another widget
        return;
    }

    TreeWidget::dragEnterEvent(event);

    dragItems.clear();

    /**
       The information of the items being dragged can be obtained from the mimeData
       of QDragEvent using the following code. However, this method only gives the
       local position (local row index) of each item and the corresponding item object
       is unknown. Therefore this method cannot be used to get the item objects being dragged.
    */
    /*
    QByteArray encoded = event->mimeData()->data("application/x-qabstractitemmodeldatalist");
    QDataStream stream(&encoded, QIODevice::ReadOnly);
    while(!stream.atEnd()){
        int row, col;
        QMap<int, QVariant> roleDataMap;
        stream >> row >> col >> roleDataMap;
        auto twItem = itemFromIndex(model()->index(row, col));
        if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
            dragItems.push_back(itwItem->item);
        }
    }
    */

    // Instead of the above method, the current selected items are used as the items being dragged.
    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>&){
            dragItems.push_back(item);
        });
}


void ItemTreeWidget::Impl::dragMoveEvent(QDragMoveEvent* event)
{
    TreeWidget::dragMoveEvent(event);

    Item* itemAtDropPosition = localRootItem;
    if(auto itwItem = dynamic_cast<ItwItem*>(itemAt(event->pos()))){
        itemAtDropPosition = itwItem->item;
    }

    Item* parentItem = nullptr;
    switch(dropIndicatorPosition()){
    case QAbstractItemView::AboveItem:
    case QAbstractItemView::BelowItem:
        parentItem = itemAtDropPosition->parentItem();
        break;
    default:
        parentItem = itemAtDropPosition;
        break;
    }
    
    for(auto& item : dragItems){
        if(!checkPositionAcceptance(item, parentItem)){
            event->ignore();
            break;
        }
    }
}


void ItemTreeWidget::Impl::dragLeaveEvent(QDragLeaveEvent *event)
{
    dragItems.clear();
}


void ItemTreeWidget::Impl::dropEvent(QDropEvent* event)
{
    isDropping = true;
    TreeWidget::dropEvent(event);
    dragItems.clear();
    isDropping = false;
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
