#include "ItemTreeWidget.h"
#include "TreeWidget.h"
#include "RootItem.h"
#include "ProjectManager.h"
#include "MenuManager.h"
#include "MessageView.h"
#include "UnifiedEditHistory.h"
#include "Archive.h"
#include <cnoid/ConnectionSet>
#include <cnoid/CloneMap>
#include <QMouseEvent>
#include <QHeaderView>
#include <QModelIndex>
#include <QBoxLayout>
#include <cnoid/stdx/optional>
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

map<string, vector<ItemTreeWidget::Impl*>> selectionSyncGroupMap;

typedef ItemTreeWidget::ItwItem ItwItem;

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
    bool isRootItemVisible;
    bool isProcessingSlotOnlocalRootItemPositionChanged;
    std::function<Item*(bool doCreate)> localRootItemUpdateFunction;
    unordered_map<Item*, ItwItem*> itemToItwItemMap;
    ItemPtr lastClickedItem;
    map<int, int> checkIdToColumnMap;
    unordered_set<Item*> itemsUnderTreeWidgetInternalOperation;

    // The use of this variable might be replaced with just blocking
    // the connections to TreeWidget's signals when calling TreeWidget's
    // functions to change its tree structure.
    int isChangingTreeWidgetTreeStructure;
    
    bool isCheckColumnShown;

    ScopedConnectionSet treeWidgetSelectionChangeConnections;
    string selectionSyncGroupId;
    vector<ItemTreeWidget::Impl*>* pSelectionSyncGroup;
    Signal<void(const ItemList<>&)> sigSelectionChanged;
    bool isControlModifierEnabledInMousePressEvent;
    bool upOrDownKeyPressed;

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

    std::function<void(MenuManager& menuManager)> rootContextMenuFunction;
    PolymorphicItemFunctionSet  contextMenuFunctions;

    UnifiedEditHistory* unifiedEditHistory;

    Impl(ItemTreeWidget* self);
    ~Impl();
    void initialize();
    Item* findOrCreateLocalRootItem(bool doCreate);
    void setLocalRootItem(Item* item, bool doUpdateTreeWidgetItems);
    void releaseFromSelectionSyncGroup();
    void setCheckColumnShown(int column, bool on);
    bool checkPositionAcceptance(Item* item, Item* parentItem) const;
    void clearTreeWidgetItems();
    void updateTreeWidgetItems();
    void expandAll(QTreeWidgetItem* twItem);
    ItwItem* findItwItem(Item* item);
    void addCheckColumn(int checkId);
    void updateCheckColumnIter(QTreeWidgetItem* twItem, int checkId, int column);
    void releaseCheckColumn(int checkId);
    void updateItemDisplay(ItwItem* itwItem);
    void applyDefaultItemDisplay(Item* item, Display& display);
    void insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItemCandidate);
    ItwItem* findNextItwItem(Item* item, bool isTopLevelItem);
    ItwItem* findNextItwItemInSubTree(Item* item, bool doTraverse);
    bool isItemUnderTreeWidgetInternalOperation(Item* item);
    void onSubTreeAddedOrMoved(Item* item);
    void onSubTreeRemoved(Item* item);
    void onItemAssigned(Item* assigned, const Item* srcItem);

    void getItemsIter(ItwItem* itwItem, ItemList<>& itemList);
    ItemList<> getSelectedItems() const;
    void selectAllItems();
    bool unselectItemsInOtherWidgetsInSelectionSyncGroup();
    void clearSelection();
    void setSelectedItemsChecked(bool on);
    void toggleSelectedItemChecks();
    void forEachTopItems(
        const ItemList<>& items, std::function<bool(Item*, unordered_set<Item*>& itemSet)> callback);
    void copySelectedItems();
    void copySelectedItemsInSubTree(Item* item, Item* duplicated, unordered_set<Item*>& itemSet, CloneMap& cloneMap);
    void copySelectedItemsWithSubTrees();
    void cutSelectedItems();
    bool pasteItems(bool doCheckPositionAcceptance);
    void onTreeWidgetRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onTreeWidgetRowsInserted(const QModelIndex& parent, int start, int end);
    void revertItemPosition(Item* item);
    void onTreeWidgetSelectionChanged();
    void updateItemSelectionIter(QTreeWidgetItem* twItem, unordered_set<Item*>& selectedItemSet);
    void onTreeWidgetCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous);
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
    std::string getViewTitle();
    View* findOwnerView();
    void restoreState(const Archive& archive, const ListingPtr expanded);
    void restoreExpandedItems(const Archive& archive, const ListingPtr expanded);
};

class ItemTreeWidget::ItwItem : public QTreeWidgetItem
{
public:
    Item* item;
    ItemTreeWidget::Impl* widgetImpl;
    ScopedConnection itemNameConnection;
    ScopedConnection itemSelectionConnection;
    ScopedConnection itemCheckConnection;
    ScopedConnection displayUpdateConnection;
    bool isExpandedBeforeRemoving;
    bool isTemporaryAttributeDisplay;

    struct DisplayState {
        QColor foregroundColor;
        bool fontItalicState;
    };
    stdx::optional<DisplayState> orgDisplayState;

    ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl);
    virtual ~ItwItem();
    virtual void setData(int column, int role, const QVariant& value) override;
};

}


ItemTreeWidget::ItwItem::ItwItem(Item* item, ItemTreeWidget::Impl* widgetImpl)
    : item(item),
      widgetImpl(widgetImpl)
{
    widgetImpl->itemToItwItemMap[item] = this;

    Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsDropEnabled;
    if(widgetImpl->isCheckColumnShown){
        flags |= Qt::ItemIsUserCheckable;
    }
    if(!item->hasAttribute(Item::Attached)){
        flags |= Qt::ItemIsEditable | Qt::ItemIsDragEnabled;
    }
    setFlags(flags);

    setToolTip(0, QString());

    setText(0, item->displayName().c_str());
    itemNameConnection =
        item->sigNameChanged().connect(
            [this](const std::string& /* oldName */){
                setText(0, this->item->displayName().c_str());
            });

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
    isTemporaryAttributeDisplay = false;

    widgetImpl->updateItemDisplay(this);
    displayUpdateConnection =
        item->sigUpdated().connect(
            [this](){ this->widgetImpl->updateItemDisplay(this); });
}


ItemTreeWidget::ItwItem::~ItwItem()
{
    widgetImpl->itemToItwItemMap.erase(item);

    if(widgetImpl->lastClickedItem == item){
        widgetImpl->lastClickedItem = nullptr;
    }
}


void ItemTreeWidget::ItwItem::setData(int column, int role, const QVariant& value)
{
    if(column == 0){
        if(role != Qt::EditRole){
            QTreeWidgetItem::setData(column, role, value);
        } else {
            // Prevent setting an empty name
            if(value.type() == QVariant::String && !value.toString().isEmpty()){
                QTreeWidgetItem::setData(column, role, value);
                item->setName(value.toString().toStdString());
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


QBrush ItemTreeWidget::Display::foreground() const
{
    return itwItem->foreground(0);
}


void ItemTreeWidget::Display::setForeground(const QBrush& brush)
{
    itwItem->setForeground(0, brush);
}


QBrush ItemTreeWidget::Display::background() const
{
    return itwItem->background(0);
}
    

void ItemTreeWidget::Display::setBackground(const QBrush& brush)
{
    itwItem->setBackground(0, brush);
}


QFont ItemTreeWidget::Display::font() const
{
    return itwItem->font(0);
}


void ItemTreeWidget::Display::setFont(const QFont& font)
{
    itwItem->setFont(0, font);
}


QIcon ItemTreeWidget::Display::icon() const
{
    return itwItem->icon(0);
}


void ItemTreeWidget::Display::setIcon(const QIcon& icon)
{
    itwItem->setIcon(0, icon);
}


void ItemTreeWidget::Display::setToolTip(const std::string& toolTip)
{
    itwItem->setText(0, toolTip.c_str());
}


void ItemTreeWidget::Display::setStatusTip(const std::string& statusTip)
{
    itwItem->setStatusTip(0, statusTip.c_str());
}


void ItemTreeWidget::Display::setNameEditable(bool on)
{
    if(on){
        itwItem->setFlags(itwItem->flags() | Qt::ItemIsEditable);
    } else {
        itwItem->setFlags(itwItem->flags() & ~Qt::ItemIsEditable);
    }
}


void ItemTreeWidget::Display::setDisabled(bool on)
{
    itwItem->setDisabled(on);
}


ItemTreeWidget::ItemTreeWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
    impl->initialize();
    
    auto box = new QVBoxLayout;
    box->setContentsMargins(0, 0, 0, 0);
    box->addWidget(impl);
    setLayout(box);
}


ItemTreeWidget::Impl::Impl(ItemTreeWidget* self)
    : self(self),
      projectRootItem(RootItem::instance()),
      projectManager(ProjectManager::instance())
{

}


ItemTreeWidget::~ItemTreeWidget()
{
    delete impl;
}


ItemTreeWidget::Impl::~Impl()
{
    clear();
    releaseFromSelectionSyncGroup();
}


void ItemTreeWidget::Impl::initialize()
{
    isRootItemVisible = false;
    isProcessingSlotOnlocalRootItemPositionChanged = false;
    isChangingTreeWidgetTreeStructure = 0;
    isCheckColumnShown = true;
    isControlModifierEnabledInMousePressEvent = false;
    upOrDownKeyPressed = false;
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
    setExpandsOnDoubleClick(false);

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
            [&](Item* assigned, const Item* srcItem){ onItemAssigned(assigned, srcItem); }));

    sigRowsAboutToBeRemoved().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsAboutToBeRemoved(parent, start, end); });
    
    sigRowsInserted().connect(
        [&](const QModelIndex& parent, int start, int end){
            onTreeWidgetRowsInserted(parent, start, end);  });

    treeWidgetSelectionChangeConnections.add(
        sigItemSelectionChanged().connect(
            [&](){ onTreeWidgetSelectionChanged(); }));

    treeWidgetSelectionChangeConnections.add(
        sigCurrentItemChanged().connect(
            [&](QTreeWidgetItem* current, QTreeWidgetItem* previous){
                onTreeWidgetCurrentItemChanged(current, previous); }));

    pSelectionSyncGroup = nullptr;
    
    projectManagerConnections.add(
        projectManager->sigProjectAboutToBeLoaded().connect(
            [&](int recursiveLevel){ onProjectAboutToBeLoaded(recursiveLevel); }));

    projectManagerConnections.add(
        projectManager->sigProjectLoaded().connect(
            [&](int recursiveLevel){ onProjectLoaded(recursiveLevel); }));

    fontPointSizeDiff = 0;

    unifiedEditHistory = UnifiedEditHistory::instance();

    self->customizeDisplay_(
        typeid(Item),
        [this](Item* item, Display& display){ applyDefaultItemDisplay(item, display); });
}


RootItem* ItemTreeWidget::projectRootItem()
{
    return impl->projectRootItem;
}



/**
  This function returns the top item of a sub tree that the item tree widget covers.
  The item is not necessarily the project root item of the RootItem type but it may
  be any item in the project item tree.
*/
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
                setLocalRootItem(item, false);
            }
        }
    }
    return localRootItem;
}


/**
   This function specifies the item that is displayed by this widget as its root tiem.
*/
void ItemTreeWidget::setRootItem(Item* item, bool doUpdateTreeWidgetItems)
{
    impl->setLocalRootItem(item, doUpdateTreeWidgetItems);
}


void ItemTreeWidget::Impl::setLocalRootItem(Item* item, bool doUpdateTreeWidgetItems)
{
    if(item != localRootItem){
        localRootItem = item;
        localRootItemConnection.disconnect();

        if(item && item != projectRootItem){
            // When the position of the local root item is changed,
            // the local root item may also be changed.
            // Check it in the following callback function.
            localRootItemConnection =
                item->sigTreePositionChanged2().connect(
                    [&,item](Item* topItem, Item* prevTopParentItem){
                        if(!isProcessingSlotOnlocalRootItemPositionChanged){
                            isProcessingSlotOnlocalRootItemPositionChanged = true;
                            if(prevTopParentItem){ // Exclude a newly added item
                                setLocalRootItem(nullptr, false);

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

        if(doUpdateTreeWidgetItems){
            updateTreeWidgetItems();
        }
    }
}


void ItemTreeWidget::setRootItemUpdateFunction(std::function<Item*(bool doCreate)> callback)
{
    impl->localRootItemUpdateFunction = callback;
    impl->localRootItem = nullptr;
}


//! \note This functions does not update the tree widget items
void ItemTreeWidget::setRootItemVisible(bool on)
{
    impl->isRootItemVisible = on;
}


bool ItemTreeWidget::isRootItemVisible() const
{
    return impl->isRootItemVisible;
}


void ItemTreeWidget::setSelectionSyncGroup(const std::string& id)
{
    impl->releaseFromSelectionSyncGroup();
    impl->selectionSyncGroupId = id;
    impl->pSelectionSyncGroup = &selectionSyncGroupMap[id];
    impl->pSelectionSyncGroup->push_back(impl);
}


void ItemTreeWidget::Impl::releaseFromSelectionSyncGroup()
{
    if(pSelectionSyncGroup){
        auto iter = std::find(pSelectionSyncGroup->begin(), pSelectionSyncGroup->end(), this);
        if(iter != pSelectionSyncGroup->end()){
            pSelectionSyncGroup->erase(iter);
        }
        if(pSelectionSyncGroup->empty()){
            selectionSyncGroupMap.erase(selectionSyncGroupId);
        }
        pSelectionSyncGroup = nullptr;
    }
}


void ItemTreeWidget::setDragDropEnabled(bool on)
{
    if(on){
        impl->setDragDropMode(QAbstractItemView::InternalMove);
    } else {
        impl->setDragDropMode(QAbstractItemView::NoDragDrop);
    }        
}


/**
   This function specifies whether or not to display the check column of each item.
   
   \note This function only changes the internal state regarding the check column.
   In order to update the actual display of the check columns, the updateTreeWidgetItems
   function must be executed.
*/
void ItemTreeWidget::setCheckColumnShown(bool on)
{
    impl->isCheckColumnShown = on;
    for(auto& kv : impl->checkIdToColumnMap){
        int column = kv.second;
        impl->setCheckColumnShown(column, on);
    }
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


bool ItemTreeWidget::checkItemVisibility(Item* item) const
{
    return impl->itemToItwItemMap.find(item) != impl->itemToItwItemMap.end();
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


void ItemTreeWidget::customizeRootContextMenu(std::function<void(MenuManager& menuManager)> func)
{
    impl->rootContextMenuFunction = func;
}


void ItemTreeWidget::updateTreeWidgetItems()
{
    impl->updateTreeWidgetItems();
}


void ItemTreeWidget::Impl::clearTreeWidgetItems()
{
    clear();
}


void ItemTreeWidget::Impl::updateTreeWidgetItems()
{
    clearTreeWidgetItems();

    if(localRootItem){
        isChangingTreeWidgetTreeStructure++;
        if(isRootItemVisible && localRootItem != projectRootItem){
            insertItem(invisibleRootItem(), localRootItem, true);
        } else {
            for(auto item = localRootItem->childItem(); item; item = item->nextItem()){
                insertItem(invisibleRootItem(), item, true);
            }
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


void ItemTreeWidget::expandAll(Item* item)
{
    if(auto itwItem = impl->findItwItem(item)){
        impl->expandAll(itwItem);
    }
}


void ItemTreeWidget::Impl::expandAll(QTreeWidgetItem* twItem)
{
    twItem->setExpanded(true);
    
    int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        if(auto childItem = twItem->child(i)){
            expandAll(childItem);
        }
    }
}


void ItemTreeWidget::editItemName(Item* item)
{
    if(auto itwItem = impl->findItwItem(item)){
        bool isEditable = (itwItem->flags() & Qt::ItemIsEditable);
        if(!isEditable){
            itwItem->setFlags(itwItem->flags() | Qt::ItemIsEditable);
        }
        impl->editItem(itwItem);
        if(!isEditable){
            itwItem->setFlags(itwItem->flags() & ~Qt::ItemIsEditable);
        }
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


void ItemTreeWidget::Impl::updateItemDisplay(ItwItem* itwItem)
{
    itemDisplay.itwItem = itwItem;
    displayFunctions.dispatch(itwItem->item);
}


void ItemTreeWidget::Impl::applyDefaultItemDisplay(Item* item, Display& display)
{
    auto& itwItem = display.itwItem;

    bool isTemporary = item->isTemporary();
    while(item->isSubItem()){
        item = item->parentItem();
        isTemporary = item->isTemporary();
    }
    
    if(isTemporary){
        if(!itwItem->isTemporaryAttributeDisplay){
            ItwItem::DisplayState state;
            auto fg = display.foreground();
            qreal h, s, v;
            state.foregroundColor = fg.color();
            state.foregroundColor.getHsvF(&h, &s, &v);
            if(v >= 0.8){
                v = 0.6;
            } else if(v <= 0.2){
                v = 0.4;
            }
            fg.setColor(QColor::fromHsvF(h, s, v));
            display.setForeground(fg);
            auto font = display.font();
            state.fontItalicState = font.italic();
            font.setItalic(true);
            display.setFont(font);
            itwItem->isTemporaryAttributeDisplay = true;
            itwItem->orgDisplayState = state;
        }
    } else {
        if(itwItem->isTemporaryAttributeDisplay){
            auto& state = *itwItem->orgDisplayState;
            auto fg = display.foreground();
            fg.setColor(state.foregroundColor);
            display.setForeground(fg);
            auto font = display.font();
            font.setItalic(state.fontItalicState);
            display.setFont(font);
            itwItem->isTemporaryAttributeDisplay = false;
        }
    }
}


void ItemTreeWidget::Impl::insertItem(QTreeWidgetItem* parentTwItem, Item* item, bool isTopLevelItemCandidate)
{
    auto itwItem = findItwItem(item);

    if(itwItem){
        parentTwItem = itwItem;

    } else {
        if(!findOrCreateLocalRootItem(false)){
            return;
        }

        bool isVisible = false;

        if(item->isOwnedBy(localRootItem) || (isRootItemVisible && (item == localRootItem))){
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
            itwItem = new ItwItem(item, this);
            bool isFirstNewChildItem = parentTwItem->childCount() == 0;
            auto nextItwItem = findNextItwItem(item, isTopLevelItemCandidate);
            if(nextItwItem){
                int index = parentTwItem->indexOfChild(nextItwItem);
                parentTwItem->insertChild(index, itwItem);
            } else {
                parentTwItem->addChild(itwItem);
            }
            if(projectLoadingWithItemExpansionInfoStack.empty() ||
               !projectLoadingWithItemExpansionInfoStack.top()){
                if(!parentTwItem->isExpanded() && isFirstNewChildItem &&
                   (!item->hasAttribute(Item::Attached) || item->hasAttribute(Item::Unique))){
                    parentTwItem->setExpanded(true);
                }
            }
            isTopLevelItemCandidate = false;
            parentTwItem = itwItem;
        }
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

    if(!nextItwItem && isTopLevelItem){
        auto upperItem = item->parentItem();
        while(upperItem){
            nextItwItem = findNextItwItemInSubTree(upperItem, true);
            if(nextItwItem){
                break;
            }
            upperItem = upperItem->parentItem();
        }
    }
    
    return nextItwItem;
}


ItwItem* ItemTreeWidget::Impl::findNextItwItemInSubTree(Item* item, bool doTraverse)
{
    ItwItem* found = nullptr;

    for(auto nextItem = item->nextItem(); nextItem; nextItem = nextItem->nextItem()){
        found = findItwItem(nextItem);
        if(found){
            break;
        }
        if(doTraverse){
            if(auto childItem = nextItem->childItem()){
                found = findItwItem(childItem);
                if(found){
                    break;
                }
                found = findNextItwItemInSubTree(childItem, true);
                if(found){
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
        }
        delete itwItem;
    } else {
        for(auto child = item->childItem(); child; child = child->nextItem()){
            onSubTreeRemoved(child);
        }
    }

    isChangingTreeWidgetTreeStructure--;
}


void ItemTreeWidget::Impl::onItemAssigned(Item* assigned, const Item* srcItem)
{
    if(findItwItem(assigned)){
        assigned->setSelected(srcItem->isSelected());
        int numCheckColumns = projectRootItem->numCheckEntries();
        for(int i=0; i < numCheckColumns; ++i){
            assigned->setChecked(i, srcItem->isChecked(i));
        }
    }
}


ItemList<> ItemTreeWidget::getItems() const
{
    ItemList<> itemList;
    impl->getItemsIter(static_cast<ItwItem*>(impl->invisibleRootItem()), itemList);
    return itemList;
}


void ItemTreeWidget::Impl::getItemsIter(ItwItem* itwItem, ItemList<>& itemList)
{
    const int n = itwItem->childCount();
    for(int i=0; i < n; ++i){
        auto child = static_cast<ItwItem*>(itwItem->child(i));
        itemList.push_back(child->item);
        if(child->childCount() > 0){
            getItemsIter(child, itemList);
        }
    }
}


SignalProxy<void(const ItemList<>&)> ItemTreeWidget::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


ItemList<> ItemTreeWidget::getSelectedItems() const
{
    return impl->getSelectedItems();
}


ItemList<> ItemTreeWidget::Impl::getSelectedItems() const
{
    auto selectedTwItems = selectedItems();
    ItemList<> items;
    items.reserve(selectedTwItems.size());
    for(auto& twItem : selectedTwItems){
        items.push_back(static_cast<ItwItem*>(twItem)->item);
    }
    return items;
}


//! \return true if the selected items are changed
bool ItemTreeWidget::selectOnly(Item* item)
{
    bool selectionChanged = false;
    
    impl->projectRootItem->beginItemSelectionChanges();
    
    selectionChanged = impl->unselectItemsInOtherWidgetsInSelectionSyncGroup();

    bool isSelected = false;
    for(auto& selected : impl->getSelectedItems()){
        if(selected == item){
            isSelected = true;
        } else {
            selected->setSelected(false);
        }
    }
    if(!isSelected){
        if(impl->findItwItem(item)){
            item->setSelected(true, true);
            selectionChanged = true;
        }
    }

    impl->projectRootItem->endItemSelectionChanges();
    
    return selectionChanged;
}


void ItemTreeWidget::selectAllItems()
{
    impl->selectAllItems();
}


void ItemTreeWidget::Impl::selectAllItems()
{
    projectRootItem->beginItemSelectionChanges();

    unselectItemsInOtherWidgetsInSelectionSyncGroup();
    
    for(auto& kv : itemToItwItemMap){
        auto& item = kv.first;
        item->setSelected(true);
    }

    projectRootItem->endItemSelectionChanges();
}


// Returns true if some items are unselected
bool ItemTreeWidget::Impl::unselectItemsInOtherWidgetsInSelectionSyncGroup()
{
    bool selectionChanged = false;
    
    if(pSelectionSyncGroup){
        vector<Item*> itemsToUnselect;
        for(auto item : projectRootItem->selectedItems()){
            bool isSelfMember = false;
            bool isAnotherMember = false;
            if(findItwItem(item)){
                isSelfMember = true;
            } else {
                for(auto& treeWidget : *pSelectionSyncGroup){
                    if(treeWidget != this){
                        if(treeWidget->findItwItem(item)){
                            isAnotherMember = true;
                            break;
                        }
                    }
                }
            }
            if(!isSelfMember && isAnotherMember){
                itemsToUnselect.push_back(item);
            }
        }
        if(!itemsToUnselect.empty()){
            for(auto& item : itemsToUnselect){
                item->setSelected(false);
            }
            selectionChanged = true;
        }
    }

    return selectionChanged;
}        


void ItemTreeWidget::clearSelection()
{
    impl->clearSelection();
}


void ItemTreeWidget::Impl::clearSelection()
{
    projectRootItem->beginItemSelectionChanges();
    for(auto& item : getSelectedItems()){
        item->setSelected(false);
    }
    projectRootItem->endItemSelectionChanges();
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
(const ItemList<>& items, std::function<bool(Item*, unordered_set<Item*>& itemSet)> callback)
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
            if(!callback(item, itemSet)){
                return;
            }
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

    CloneMap cloneMap;

    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>& itemSet){
            if(auto clone = cloneMap.getClone(item)){
                copiedItems.push_back(clone);
                copySelectedItemsInSubTree(item, clone, itemSet, cloneMap);
            }
            return true;
        });

    cloneMap.replacePendingObjects();
}


void ItemTreeWidget::Impl::copySelectedItemsInSubTree
(Item* item, Item* clone, unordered_set<Item*>& itemSet, CloneMap& cloneMap)
{
    for(Item* child = item->childItem(); child; child = child->nextItem()){
        ItemPtr childClone;
        if(itemSet.find(child) != itemSet.end() || child->hasAttribute(Item::Attached)){
            if(child->isSubItem()){
                childClone = cloneMap.findClone<Item>(child);
                /*
                if(!childClone){
                    childCLone = clone->findChildItem(child->name());
                }
                */
            } else {
                childClone = cloneMap.getClone(child);
                if(childClone){
                    clone->addChildItem(childClone);
                }
            }
        }
        if(childClone){
            copySelectedItemsInSubTree(child, childClone, itemSet, cloneMap);
        } else {
            copySelectedItemsInSubTree(child, childClone, itemSet, cloneMap);
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

    CloneMap cloneMap;

    forEachTopItems(
        getSelectedItems(),
        [&](Item* item, unordered_set<Item*>&){
            if(auto clone = item->cloneSubTree(cloneMap)){
                copiedItems.push_back(clone);
            }
            return true;
        });

    cloneMap.replacePendingObjects();
}


void ItemTreeWidget::cutSelectedItems()
{
    impl->cutSelectedItems();
}


void ItemTreeWidget::Impl::cutSelectedItems()
{
    auto selectedItems = getSelectedItems();

    bool cuttable = true;
    forEachTopItems(
        selectedItems,
        [&](Item* item, unordered_set<Item*>&){
            if(!self->checkCuttable(item)){
                cuttable = false;
                return false;
            }
            return true;
        });

    if(!cuttable){
        return;
    }

    copiedItems.clear();

    bool editGroupCreated = false;
    if(selectedItems.size() > 1){
        unifiedEditHistory->beginEditGroup(format(_("Cut items in {0}"), getViewTitle()));
        editGroupCreated = true;
    }
    
    forEachTopItems(
        selectedItems,
        [&](Item* item, unordered_set<Item*>&){
            copiedItems.push_back(item);
            item->removeFromParentItem();
            return true;
        });

    if(editGroupCreated){
        unifiedEditHistory->endEditGroup();
    }
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
            if(!self->checkPastable(parentItem)){
                showWarningDialog(
                    format(_("The copied items cannot be pasted to \"{0}\"."),
                           parentItem->displayName()));
                isPastable = false;
            }
        }
        if(isPastable){
            bool editGroupCreated = false;
            if(copiedItems.size() > 1){
                unifiedEditHistory->beginEditGroup(format(_("Paste items in {0}"), getViewTitle()), false);
                editGroupCreated = true;
            }
            CloneMap cloneMap;
            auto it = copiedItems.begin();
            while(it != copiedItems.end()){
                auto& item = *it;
                auto clone = item->cloneSubTree(cloneMap);
                cloneMap.replacePendingObjects();
                if(clone){
                    parentItem->addChildItem(clone, true);
                    ++it;
                } else {
                    parentItem->addChildItem(item, true);
                    it = copiedItems.erase(it);
                }
                pasted = true;
            }
            if(editGroupCreated){
                unifiedEditHistory->endEditGroup();
            }
        }
    }
    return pasted;
}


ItemList<Item> ItemTreeWidget::getCopiedItems()
{
    return impl->copiedItems;
}


bool ItemTreeWidget::checkCuttable(Item* item) const
{
    return !item->hasAttribute(Item::Attached);
}


bool ItemTreeWidget::checkCopiable(Item* item) const
{
    return !item->hasAttribute(Item::Unique);
}


bool ItemTreeWidget::checkPastable(Item* pasteParentItem) const
{
    if(impl->copiedItems.empty()){
        return false;
    }
    for(auto& item : impl->copiedItems){
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
        item->removeFromParentItem();
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

    vector<Item*> canceledItems;
    for(auto item : items){
        if(!parentItem->insertChild(nextItem, item, true)){
            canceledItems.push_back(item);
        }
    }

    itemsUnderTreeWidgetInternalOperation.clear();

    for(auto item : canceledItems){
        revertItemPosition(item);
    }
}


void ItemTreeWidget::Impl::revertItemPosition(Item* item)
{
    if(auto itwItem = findItwItem(item)){

        isChangingTreeWidgetTreeStructure++;

        QTreeWidgetItem* currentParentTwItem = itwItem->parent();
        if(!currentParentTwItem){
            currentParentTwItem = invisibleRootItem();
        }
        currentParentTwItem->removeChild(itwItem);

        auto parentItem = item->parentItem();
        bool isTopLevelItem = false;
        QTreeWidgetItem* trueParentTwItem = findItwItem(parentItem);
        if(!trueParentTwItem){
            trueParentTwItem = invisibleRootItem();
            isTopLevelItem = true;
        }
        auto nextItwItem = findNextItwItem(item, isTopLevelItem);
        if(nextItwItem){
            int index = trueParentTwItem->indexOfChild(nextItwItem);
            trueParentTwItem->insertChild(index, itwItem);
        } else {
            trueParentTwItem->addChild(itwItem);
        }
        
        isChangingTreeWidgetTreeStructure--;
    }
}


void ItemTreeWidget::Impl::onTreeWidgetSelectionChanged()
{
    ItemList<> items;
    auto selectedTwItems = selectedItems();
    bool doEmitSignal = sigSelectionChanged.hasConnections();
    if(doEmitSignal){
        items.reserve(selectedTwItems.size());
    }
    unordered_set<Item*> selectedItemSet;
    for(auto& twItem : selectedTwItems){
        if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
            auto item = itwItem->item;
            selectedItemSet.insert(item);
            if(doEmitSignal){
                items.push_back(item);
            }
        }
    }

    projectRootItem->beginItemSelectionChanges();

    if(!isControlModifierEnabledInMousePressEvent){
        unselectItemsInOtherWidgetsInSelectionSyncGroup();
    }

    updateItemSelectionIter(invisibleRootItem(), selectedItemSet);

    if(!upOrDownKeyPressed){
        projectRootItem->endItemSelectionChanges();
    }

    if(doEmitSignal){
        sigSelectionChanged(items);
    }
}


void ItemTreeWidget::Impl::updateItemSelectionIter(QTreeWidgetItem* twItem, unordered_set<Item*>& selectedItemSet)
{
    if(auto itwItem = dynamic_cast<ItwItem*>(twItem)){
        auto item = itwItem->item;
        bool on = selectedItemSet.find(item) != selectedItemSet.end();
        bool doUpdate = (on != item->isSelected());
        bool isCurrent = (item == lastClickedItem) && on;
        if(!doUpdate){
            if(isCurrent && (item != projectRootItem->currentItem())){
                doUpdate = true;
            }
        }
        if(doUpdate){
            itwItem->itemSelectionConnection.block();
            item->setSelected(on, isCurrent);
            itwItem->itemSelectionConnection.unblock();
        }
    }
    int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        updateItemSelectionIter(twItem->child(i), selectedItemSet);
    }
}


/**
   When the selection changed with the cursor keys, the newly selected item should be
   the current item, and the sigCurrentItemChanged signal is the only way to detect it.
*/
void ItemTreeWidget::Impl::onTreeWidgetCurrentItemChanged(QTreeWidgetItem* current, QTreeWidgetItem* previous)
{
    auto selected = selectedItems();
    if(selected.size() == 1 && current == selected.front()){
        if(auto itwItem = dynamic_cast<ItwItem*>(current)){
            itwItem->itemSelectionConnection.block();
            itwItem->item->setSelected(true, true);
            itwItem->itemSelectionConnection.unblock();
        }
    }
    if(upOrDownKeyPressed){
        // End the changes begun in the onTreeWidgetSelectionChanged function
        projectRootItem->endItemSelectionChanges();
    }
    upOrDownKeyPressed = false;
}


void ItemTreeWidget::Impl::setItwItemSelected(ItwItem* itwItem, bool on)
{
    if(on != itwItem->isSelected()){
        treeWidgetSelectionChangeConnections.block();
        itwItem->setSelected(on);
        treeWidgetSelectionChangeConnections.unblock();

        if(sigSelectionChanged.hasConnections()){
            sigSelectionChanged(getSelectedItems());
        }
    }
}


void ItemTreeWidget::Impl::toggleItwItemCheck(ItwItem* itwItem, int checkId, bool on)
{
    auto state = itwItem->checkState(checkId + 1);
    if(((state != Qt::Checked) && on) || ((state != Qt::Unchecked) && !on)){
        itwItem->setCheckState(checkId + 1, on ? Qt::Checked : Qt::Unchecked);
    }
}


void ItemTreeWidget::Impl::zoomFontSize(int pointSizeDiff)
{
    QFont font = TreeWidget::font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    setFont(font);
    fontPointSizeDiff += pointSizeDiff;

    int defaultIconSize = style()->pixelMetric(QStyle::PM_SmallIconSize);
    QFontInfo info(font);
    int fontSize = info.pixelSize();
    if(fontSize > defaultIconSize){
        setIconSize(QSize(fontSize, fontSize));
    } else {
        setIconSize(QSize(defaultIconSize, defaultIconSize));
    }
}


void ItemTreeWidget::Impl::mousePressEvent(QMouseEvent* event)
{
    ItwItem* itwItem = dynamic_cast<ItwItem*>(itemAt(event->pos()));
    lastClickedItem = nullptr;

    if(event->modifiers() & Qt::ControlModifier){
        isControlModifierEnabledInMousePressEvent = true;
    }

    // Emit sigSelectionChanged when clicking on an already selected item
    if(event->button() == Qt::LeftButton){
        auto selected = selectedItems();
        if(itwItem){
            if(selected.size() == 1 && itwItem == selected.front()){
                itwItem->itemSelectionConnection.block();
                unselectItemsInOtherWidgetsInSelectionSyncGroup();
                itwItem->item->setSelected(true, true);
                itwItem->itemSelectionConnection.unblock();
            } else {
                lastClickedItem = itwItem->item;
            }
        }
    }
    
    TreeWidget::mousePressEvent(event);

    isControlModifierEnabledInMousePressEvent = false;

    if(event->button() == Qt::RightButton){
        menuManager.setNewPopupMenu(this);
        if(itwItem){
            contextMenuFunctions.dispatch(itwItem->item);
        } else {
            if(rootContextMenuFunction){
                clearSelection();
                rootContextMenuFunction(menuManager);
            }
        }
        if(menuManager.numItems() > 0){
            menuManager.popupMenu()->popup(event->globalPos());
        }
    }
}


void ItemTreeWidget::Impl::keyPressEvent(QKeyEvent* event)
{
    bool processed = false;

    if(event->modifiers() == Qt::NoModifier){
        switch(event->key()){
        case Qt::Key_Up:
        case Qt::Key_Down:
            upOrDownKeyPressed = true;
            break;
            
        case Qt::Key_Escape:
            clearSelection();
            processed = true;
            break;
        case Qt::Key_Delete:
            cutSelectedItems();
            processed = true;
            break;
        default:
            break;
        }
    }
        
    if(!processed && (event->modifiers() & Qt::ControlModifier)){

        processed = true;

        switch(event->key()){

        case Qt::Key_A:
            selectAllItems();
            break;
            
        case Qt::Key_R:
            unifiedEditHistory->flushNewRecordBuffer();
            for(auto& item : getSelectedItems()){
                item->reload();
            }
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
            return true;
        });
}


void ItemTreeWidget::Impl::dragMoveEvent(QDragMoveEvent* event)
{
    TreeWidget::dragMoveEvent(event);

    Item* parentItem = nullptr;

    Item* itemAtDropPosition = localRootItem;
    if(auto itwItem = dynamic_cast<ItwItem*>(itemAt(event->pos()))){
        itemAtDropPosition = itwItem->item;
    }
    if(itemAtDropPosition){
        switch(dropIndicatorPosition()){
        case QAbstractItemView::AboveItem:
        case QAbstractItemView::BelowItem:
            parentItem = itemAtDropPosition->parentItem();
            break;
        default:
            parentItem = itemAtDropPosition;
            break;
        }
    }
    
    if(!parentItem){
        event->ignore();
    } else {
        for(auto& item : dragItems){
            if(!checkPositionAcceptance(item, parentItem)){
                event->ignore();
                break;
            }
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

    bool editGroupCreated = false;
    if(dragItems.size() > 1){
        unifiedEditHistory->beginEditGroup(format(_("Drop items in {0}"), getViewTitle()));
        editGroupCreated = true;
    }
    
    TreeWidget::dropEvent(event);
    dragItems.clear();

    if(editGroupCreated){
        unifiedEditHistory->endEditGroup();
    }
    
    isDropping = false;
}


std::string ItemTreeWidget::Impl::getViewTitle()
{
    string viewTitle;
    if(auto view = findOwnerView()){
        viewTitle = format(_("the {0} view"), view->windowTitle().toStdString());
    } else {
        viewTitle = "a view";
    }
    return viewTitle;
}


View* ItemTreeWidget::Impl::findOwnerView()
{
    View* view = nullptr;
    auto widget = self->parentWidget();
    while(widget){
        view = dynamic_cast<View*>(widget);
        if(view){
            break;
        }
        widget = widget->parentWidget();
    }
    return view;
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
                if(auto id = archive.getItemIdNode(itwItem->item)){
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

                /**
                   The selection states of the child items must be restored here
                   because a tree item is not actually selected if the item is
                   initially inserted to an unexpanded parent item.
                */
                int n = itwItem->childCount();
                for(int i=0; i < n; ++i){
                    auto childItwItem = static_cast<ItwItem*>(itwItem->child(i));
                    if(childItwItem->item->isSelected()){
                        setItwItemSelected(childItwItem, true);
                    }
                }
            }
        }
    }
}
