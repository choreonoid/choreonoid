/**
   @author Shin'ichiro Nakaoka
*/

#include "ItemTreeView.h"
#include "Item.h"
#include "RootItem.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "AppConfig.h"
#include "Archive.h"
#include "TreeWidget.h"
#include <cnoid/ConnectionSet>
#include <QBoxLayout>
#include <QApplication>
#include <QMouseEvent>
#include <QHeaderView>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>
#include <set>
#include <cassert>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

ItemTreeView* itemTreeView = 0;

typedef Signal<void(bool isChecked)> SigCheckToggled;
typedef boost::shared_ptr<SigCheckToggled> SigCheckToggledPtr;

// Item of the item tree view
class ItvItem : public QTreeWidgetItem
{
public:
    ItvItem(Item* item, ItemTreeViewImpl* itemTreeViewImpl);
    virtual ~ItvItem();
    virtual QVariant data(int column, int role) const;
    virtual void setData(int column, int role, const QVariant& value);

    SigCheckToggled* sigCheckToggled(int id){
        if(id < sigCheckToggledList.size()){
            return sigCheckToggledList[id].get();
        }
        return 0;
    }
    SigCheckToggled& getOrCreateSigCheckToggled(int id){
        if(id >= sigCheckToggledList.size()){
            sigCheckToggledList.resize(id + 1);
        }
        if(!sigCheckToggledList[id]){
            sigCheckToggledList[id] = boost::make_shared<SigCheckToggled>();
        }
        return *sigCheckToggledList[id];
    }

    ItemPtr item;
    ItemTreeViewImpl* itemTreeViewImpl;
    bool isExpandedBeforeRemoving;
private:
    vector<SigCheckToggledPtr> sigCheckToggledList;
};

// Preserved as custom data in an item
class ItvItemRef : public Referenced
{
public:
    ItvItemRef(ItvItem* itvItem) : itvItem(itvItem) { }
    ItvItem* itvItem;
};

class CheckColumn : public Referenced
{
public:
    ItemList<> checkedItemList;
    bool needToUpdateCheckedItemList;
    Signal<void(Item* item, bool isChecked)> sigCheckToggled;
    QString tooltip;

    CheckColumn() {
        needToUpdateCheckedItemList = true;
    }
};

typedef ref_ptr<CheckColumn> CheckColumnPtr;
}


namespace cnoid {

class ItemTreeViewImpl : public TreeWidget
{
public:
    ItemTreeViewImpl(ItemTreeView* self, RootItem* rootItem, bool showRoot);
    ~ItemTreeViewImpl();

    ItemTreeView* self;
    RootItemPtr rootItem;

    int isProceccingSlotForRootItemSignals;
    ConnectionSet connectionsFromRootItem;

    vector<CheckColumnPtr> checkColumns;

    Signal<void(Item* item, bool isChecked)> sigCheckToggledForInvalidId;
    Signal<void(bool isChecked)> sigCheckToggledForInvalidItem;
    Signal<void(const ItemList<>&)> sigSelectionChanged;
    Signal<void(const ItemList<>&)> sigSelectionOrTreeChanged;

    ItemList<> emptyItemList;
    ItemList<> selectedItemList;
    ItemList<Item> copiedItemList;
    Menu* popupMenu;
    MenuManager menuManager;

    bool isDropping;
    int fontPointSizeDiff;

    int addCheckColumn();
    void initializeCheckState(QTreeWidgetItem* item, int column);
    void updateCheckColumnToolTipIter(QTreeWidgetItem* item, int column, const QString& tooltip);
    void showCheckColumn(int id, bool on);
    void releaseCheckColumn(int id);
        
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void keyPressEvent(QKeyEvent* event);
        
    ItvItem* getItvItem(Item* item);
    ItvItem* getOrCreateItvItem(Item* item);
    void onSubTreeAddedOrMoved(Item* item);
    void insertItem(QTreeWidgetItem* parentTwItem, Item* item, Item* nextItem);
    void onSubTreeRemoved(Item* item, bool isMoving);
    void onTreeChanged();
    void onItemAssigned(Item* assigned, Item* srcItem);

    virtual void dropEvent(QDropEvent* event);
        
    void onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end);
    void onRowsInserted(const QModelIndex& parent, int start, int end);
    virtual bool dropMimeData(QTreeWidgetItem* parent, int index, const QMimeData* data, Qt::DropAction action);
    void onSelectionChanged();
    bool isItemSelected(Item* item);
    bool selectItem(Item* item, bool select);
    bool isItemChecked(Item* item, int id);
    bool checkItem(Item* item, bool checked, int id);
    bool extractSelectedItemsOfSubTreeTraverse(Item* item, ItemList<>* io_items);
    ItemList<>& checkedItems(int id);
    void extractCheckedItems(QTreeWidgetItem* twItem, int column, ItemList<>& checkdItems);
    void forEachTopItems(const ItemList<>& orgItemList, boost::function<void(Item*)> callback);
    void cutSelectedItems();
    void copySelectedItems();
    void copySelectedItemsSub(Item* item, ItemPtr& duplicated, set<Item*>& items);
    void copySelectedItemsWithChildren();
    void addCopiedItemToCopiedItemList(Item* item);
    void pasteItems();
    void moveCutItemsToCopiedItemList(Item* item);
    void checkSelectedItems(bool on);
    void toggleSelectedItemChecks();
    bool storeState(Archive& archive);
    void storeItemIds(Archive& archive, const char* key, const ItemList<>& items);
    bool restoreState(const Archive& archive);
    bool restoreItemStates(const Archive& archive, const char* key, boost::function<void(ItemPtr)> stateChangeFunc);
    void storeExpandedItems(Archive& archive);
    void storeExpandedItemsSub(QTreeWidgetItem* parentTwItem, Archive& archive, ListingPtr& expanded);
    void restoreExpandedItems(const Archive& archive);
    void zoomFontSize(int pointSizeDiff);
};
}


ItvItem::ItvItem(Item* item, ItemTreeViewImpl* itemTreeViewImpl)
    : item(item),
      itemTreeViewImpl(itemTreeViewImpl)
{
    setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsUserCheckable | Qt::ItemIsDropEnabled);

    if(!item->isSubItem()){
        setFlags(flags() | Qt::ItemIsEditable | Qt::ItemIsDragEnabled);
    }

    setToolTip(0, QString());

    vector<CheckColumnPtr>& checkColumns = itemTreeViewImpl->checkColumns;
    const int n = checkColumns.size();
    for(size_t i=0; i < n; ++i){
        setCheckState(i + 1, Qt::Unchecked);
        setToolTip(i + 1, checkColumns[i]->tooltip);
    }
    
    ItvItemRef* ref = new ItvItemRef(this);
    item->setCustomData(0, ref);

    isExpandedBeforeRemoving = false;
}


ItvItem::~ItvItem()
{
    item->clearCustomData(0);
}


QVariant ItvItem::data(int column, int role) const
{
    if((role == Qt::DisplayRole || role == Qt::EditRole) && column == 0){
        return item->name().c_str();
    }
    return QTreeWidgetItem::data(column, role);
}


void ItvItem::setData(int column, int role, const QVariant& value)
{
    QTreeWidgetItem::setData(column, role, value);
    
    if(column == 0){
        if(role == Qt::DisplayRole || role == Qt::EditRole){
            if(value.type() == QVariant::String){
                if(!value.toString().isEmpty()){
                    item->setName(value.toString().toStdString());
                }
            }
        }
    } else if(column >= 1 && role == Qt::CheckStateRole){
        const int id = column - 1;
        if(id < itemTreeViewImpl->checkColumns.size()){
            CheckColumnPtr& cc = itemTreeViewImpl->checkColumns[id];
            cc->needToUpdateCheckedItemList = true;
            const bool checked = ((Qt::CheckState)value.toInt() == Qt::Checked);
            cc->sigCheckToggled(item.get(), checked);
            SigCheckToggled* sig = sigCheckToggled(id);
            if(sig){
                (*sig)(checked);
            }
        }
    }
}


void ItemTreeView::initializeClass(ExtensionManager* ext)
{
    itemTreeView = ext->viewManager().registerClass<ItemTreeView>(
        "ItemTreeView", N_("Items"), ViewManager::SINGLE_DEFAULT);
}


ItemTreeView* ItemTreeView::instance()
{
    return itemTreeView;
}


ItemTreeView* ItemTreeView::mainInstance()
{
    return itemTreeView;
}


ItemTreeView::ItemTreeView()
{
    construct(RootItem::instance(), false);
}


ItemTreeView::ItemTreeView(RootItem* rootItem, bool showRoot)
{
    construct(rootItem, showRoot);
}


void ItemTreeView::construct(RootItem* rootItem, bool showRoot)
{
    setDefaultLayoutArea(View::LEFT);
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    
    impl = new ItemTreeViewImpl(this, rootItem, showRoot);

    QVBoxLayout* layout = new QVBoxLayout();
    layout->addWidget(impl);
    setLayout(layout);
}


ItemTreeViewImpl::ItemTreeViewImpl(ItemTreeView* self, RootItem* rootItem, bool showRoot)
    : self(self),
      rootItem(rootItem)
{
    using boost::bind;
    
    isProceccingSlotForRootItemSignals = 0;
    isDropping = false;
    
    setColumnCount(1);

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    header()->setResizeMode(0, QHeaderView::Stretch);
    header()->setMinimumSectionSize(0);
#else
    header()->setSectionResizeMode(0, QHeaderView::Stretch);
    //header()->setMinimumSectionSize(0);
#endif

    // default check column
    addCheckColumn();
    header()->setStretchLastSection(false);
    header()->swapSections(0, 1);

    setWordWrap(true);
    setFrameShape(QFrame::NoFrame);
    setHeaderHidden(true);
    setIndentation(12);
    setSelectionMode(QAbstractItemView::ExtendedSelection);
    setDragDropMode(QAbstractItemView::InternalMove);

    sigItemSelectionChanged().connect(bind(&ItemTreeViewImpl::onSelectionChanged, this));

    connectionsFromRootItem.add(
        rootItem->sigSubTreeAdded().connect(bind(&ItemTreeViewImpl::onSubTreeAddedOrMoved, this, _1)));
    connectionsFromRootItem.add(
        rootItem->sigSubTreeMoved().connect(bind(&ItemTreeViewImpl::onSubTreeAddedOrMoved, this, _1)));
    connectionsFromRootItem.add(
        rootItem->sigSubTreeRemoved().connect(bind(&ItemTreeViewImpl::onSubTreeRemoved, this, _1, _2)));
    connectionsFromRootItem.add(
        rootItem->sigTreeChanged().connect(bind(&ItemTreeViewImpl::onTreeChanged, this)));
    connectionsFromRootItem.add(
        rootItem->sigItemAssigned().connect(bind(&ItemTreeViewImpl::onItemAssigned, this, _1, _2)));

    QObject::connect(model(), SIGNAL(rowsAboutToBeRemoved(const QModelIndex&, int, int)),
                     self, SLOT(onRowsAboutToBeRemoved(const QModelIndex&, int, int)));
    QObject::connect(model(), SIGNAL(rowsInserted(const QModelIndex&, int, int)),
                     self, SLOT(onRowsInserted(const QModelIndex&, int, int)));

    popupMenu = new Menu(this);
    menuManager.setTopMenu(popupMenu);

    menuManager.addItem(_("Cut"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::cutSelectedItems, this));
    menuManager.addItem(_("Copy (single)"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::copySelectedItems, this));
    menuManager.addItem(_("Copy (sub tree)"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::copySelectedItemsWithChildren, this));
    menuManager.addItem(_("Paste"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::pasteItems, this));

    menuManager.addSeparator();
    menuManager.addItem(_("Check"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::checkSelectedItems, this, true));
    menuManager.addItem(_("Uncheck"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::checkSelectedItems, this, false));
    menuManager.addItem(_("Toggle checks"))
        ->sigTriggered().connect(bind(&ItemTreeViewImpl::toggleSelectedItemChecks, this));

    menuManager.addSeparator();
    menuManager.addItem(_("Select all"))
        ->sigTriggered().connect(bind(&ItemTreeView::selectAllItems, self));
    menuManager.addItem(_("Clear selection"))
        ->sigTriggered().connect(bind(&ItemTreeView::clearSelection, self));
    
    fontPointSizeDiff = 0;
    MappingPtr config = AppConfig::archive()->openMapping("ItemTreeView");
    int storedFontPointSizeDiff;
    if(config->read("fontZoom", storedFontPointSizeDiff)){
        zoomFontSize(storedFontPointSizeDiff);
    }
}


ItemTreeView::~ItemTreeView()
{

}


ItemTreeViewImpl::~ItemTreeViewImpl()
{
    // On Windows + VC++, boost::signal::trackable, the super class of
    // ItemTreeViewImpl, does not seem to work correctly, and the connection
    // is not disconnected and the program aborted by segmentation fault.
    // So the following explicit disconnection code is added.

    //connectionClassUnregistered.disconnect();
}


int ItemTreeView::addCheckColumn()
{
    return impl->addCheckColumn();
}


int ItemTreeViewImpl::addCheckColumn()
{
    //! \todo The released columns should be reused
    
    int id = checkColumns.size();
    const int n = id + 1;
    checkColumns.resize(n);
    setColumnCount(n + 1);

    checkColumns[id] = new CheckColumn;

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    header()->setResizeMode(id + 1, QHeaderView::ResizeToContents);
#else
    header()->setSectionResizeMode(id + 1, QHeaderView::ResizeToContents);
#endif

    initializeCheckState(invisibleRootItem(), id + 1);
    
    return id;
}


void ItemTreeViewImpl::initializeCheckState(QTreeWidgetItem* item, int column)
{
    item->setCheckState(column, Qt::Unchecked);
    const int n = item->childCount();
    for(int i=0; i < n; ++i){
        initializeCheckState(item->child(i), column);
    }
}


void ItemTreeView::setCheckColumnToolTip(int id, const QString& whatsThis)
{
    if(id > 0 && id < impl->checkColumns.size()){
        impl->checkColumns[id]->tooltip = whatsThis;
        updateCheckColumnToolTip(id);
    }
}


void ItemTreeView::updateCheckColumnToolTip(int id)
{
    if(id > 0 && id < impl->checkColumns.size()){
        impl->updateCheckColumnToolTipIter(
            impl->invisibleRootItem(), id + 1, impl->checkColumns[id]->tooltip);
    }
}


void ItemTreeViewImpl::updateCheckColumnToolTipIter(QTreeWidgetItem* item, int column, const QString& tooltip)
{
    item->setToolTip(column, tooltip);
    const int n = item->childCount();
    for(int i=0; i < n; ++i){
        updateCheckColumnToolTipIter(item->child(i), column, tooltip);
    }
}


void ItemTreeView::showCheckColumn(int id, bool on)
{
    impl->showCheckColumn(id, on);
}


void ItemTreeViewImpl::showCheckColumn(int id, bool on)
{
    if(id > 0 && id < checkColumns.size()){
        if(!on){
            header()->hideSection(id + 1);
        } else {
            CheckColumnPtr& cc = checkColumns[id];
            if(cc){
                header()->showSection(id + 1);
            }
        }
    }
}


void ItemTreeView::releaseCheckColumn(int id)
{
    impl->releaseCheckColumn(id);
}


void ItemTreeViewImpl::releaseCheckColumn(int id)
{
    //! todo Release the column actually

    // just hide the column currently
    showCheckColumn(id, false);
}


RootItem* ItemTreeView::rootItem()
{
    return impl->rootItem.get();
}


void ItemTreeView::showRoot(bool show)
{

}


void ItemTreeViewImpl::mousePressEvent(QMouseEvent* event)
{
    TreeWidget::mousePressEvent(event);

    if(event->button() == Qt::RightButton){
        popupMenu->popup(event->globalPos());
    }
}


void ItemTreeViewImpl::keyPressEvent(QKeyEvent* event)
{
    bool processed = true;

    switch(event->key()){
    case Qt::Key_Escape:
        self->clearSelection();
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
            self->selectAllItems();
            break;
        case Qt::Key_R:
            ItemManager::reloadItems(selectedItemList);
            break;
        case Qt::Key_Plus:
        case Qt::Key_Semicolon:
            zoomFontSize(1);
            break;
        case Qt::Key_Minus:
            zoomFontSize(-1);
            break;
        defaut:
            processed = false;
            break;
        }
    }

    if(!processed){
        TreeWidget::keyPressEvent(event);
    }
}


ItvItem* ItemTreeViewImpl::getItvItem(Item* item)
{
    ItvItem* itvItem = 0;
    ItvItemRef* ref = dynamic_cast<ItvItemRef*>(item->customData(0));
    if(ref){
        itvItem = ref->itvItem;
    }
    return itvItem;
}


ItvItem* ItemTreeViewImpl::getOrCreateItvItem(Item* item)
{
    ItvItem* itvItem = getItvItem(item);
    if(!itvItem){
        itvItem = new ItvItem(item, this);
    }
    return itvItem;
}


void ItemTreeViewImpl::onSubTreeAddedOrMoved(Item* item)
{
    isProceccingSlotForRootItemSignals++;
    
    Item* parentItem = item->parentItem();
    if(parentItem){
        if(parentItem == rootItem){
            insertItem(invisibleRootItem(), item, item->nextItem());
        } else {
            ItvItem* parentItvItem = getItvItem(parentItem);
            if(parentItvItem){
                insertItem(parentItvItem, item, item->nextItem());
            }
        }
    }

    isProceccingSlotForRootItemSignals--;
}


void ItemTreeViewImpl::insertItem(QTreeWidgetItem* parentTwItem, Item* item, Item* nextItem)
{
    ItvItem* itvItem = getOrCreateItvItem(item);
    bool inserted = false;
    if(nextItem){
        ItvItem* nextItvItem = getItvItem(nextItem);
        if(nextItvItem){
            int index = parentTwItem->indexOfChild(nextItvItem);
            parentTwItem->insertChild(index, itvItem);
            inserted = true;
        }
    }
    if(!inserted){
        parentTwItem->addChild(itvItem);
    }
        
    if(!parentTwItem->isExpanded()){
        if(!item->isSubItem()){
            parentTwItem->setExpanded(true);
        }
    }

    for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        insertItem(itvItem, childItem, 0);
    }
}


void ItemTreeViewImpl::onSubTreeRemoved(Item* item, bool isMoving)
{
    isProceccingSlotForRootItemSignals++;
    
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        QTreeWidgetItem* parentTwItem = itvItem->parent();
        if(parentTwItem){
            parentTwItem->removeChild(itvItem);
        } else {
            takeTopLevelItem(indexOfTopLevelItem(itvItem));
        }
        delete itvItem;
    }

    isProceccingSlotForRootItemSignals--;
}


void ItemTreeViewImpl::dropEvent(QDropEvent* event)
{
    isDropping = true;
    TreeWidget::dropEvent(event);
    isDropping = false;
}
    

void ItemTreeView::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    if(impl->isProceccingSlotForRootItemSignals == 0){
        impl->onRowsAboutToBeRemoved(parent, start, end);
    }
}


void ItemTreeViewImpl::onRowsAboutToBeRemoved(const QModelIndex& parent, int start, int end)
{
    connectionsFromRootItem.block();

    QTreeWidgetItem* parentTwItem = itemFromIndex(parent);
    if(!parentTwItem){
        parentTwItem = invisibleRootItem();
    }

    for(int i=start; i <= end; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            itvItem->isExpandedBeforeRemoving = itvItem->isExpanded();
            if(!isDropping){
                ItemPtr& item = itvItem->item;
                if(!item->isSubItem()){
                    item->detachFromParentItem();
                }
            }
        }
    }
    
    connectionsFromRootItem.unblock();
}


void ItemTreeView::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    if(impl->isProceccingSlotForRootItemSignals == 0){
        impl->onRowsInserted(parent, start, end);
    }
}


void ItemTreeViewImpl::onRowsInserted(const QModelIndex& parent, int start, int end)
{
    connectionsFromRootItem.block();

    QTreeWidgetItem* parentTwItem = itemFromIndex(parent);
    if(parentTwItem){
        parentTwItem->setExpanded(true);
    } else {
        parentTwItem = invisibleRootItem();
    }

    ItvItem* parentItvItem = dynamic_cast<ItvItem*>(parentTwItem);
    Item* parentItem = parentItvItem ? parentItvItem->item.get() : rootItem.get();

    ItemPtr nextItem = 0;
    if(end + 1 < parentTwItem->childCount()){
        ItvItem* nextItvItem = dynamic_cast<ItvItem*>(parentTwItem->child(end + 1));
        if(nextItvItem){
            nextItem = nextItvItem->item;
        }
    }
    
    for(int i=start; i <= end; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            ItemPtr& item = itvItem->item;
            if(!item->isSubItem()){
                parentItem->insertChildItem(item, nextItem, true);
            }
            if(itvItem->isExpandedBeforeRemoving){
                itvItem->setExpanded(true);
            }
        }
    }
    
    connectionsFromRootItem.unblock();
}


void ItemTreeViewImpl::onTreeChanged()
{

}


void ItemTreeViewImpl::onItemAssigned(Item* assigned, Item* srcItem)
{
    ItvItem* itvItem = getItvItem(assigned);
    ItvItem* srcItvItem = getItvItem(srcItem);
    if(itvItem && srcItvItem){
        for(size_t i=0; i < checkColumns.size(); ++i){
            itvItem->setCheckState(i + 1, srcItvItem->checkState(i + 1));
        }
        QModelIndex index = indexFromItem(itvItem);
        QModelIndex srcIndex = indexFromItem(srcItvItem);
        selectionModel()->select(
            index, srcItvItem->isSelected() ? QItemSelectionModel::Select : QItemSelectionModel::Deselect);
    }
}


bool ItemTreeViewImpl::dropMimeData(QTreeWidgetItem* parent, int index, const QMimeData* data, Qt::DropAction action)
{
    return TreeWidget::dropMimeData(parent, index, data, action);
}


void ItemTreeViewImpl::onSelectionChanged()
{
    selectedItemList.clear();

    QList<QTreeWidgetItem*> selected = selectedItems();
    for(int i=0; i < selected.size(); ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(selected[i]);
        if(itvItem){
            selectedItemList.push_back(itvItem->item.get());
        }
    }

    sigSelectionChanged(selectedItemList);
    sigSelectionOrTreeChanged(selectedItemList);
}


bool ItemTreeView::isItemSelected(Item* item)
{
    return impl->isItemSelected(item);
}


bool ItemTreeViewImpl::isItemSelected(Item* item)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        return itvItem->isSelected();
    }
    return false;
}


bool ItemTreeView::selectItem(Item* item, bool select)
{
    return impl->selectItem(item, select);
}


void ItemTreeView::unselectItem(Item* item)
{
    impl->selectItem(item, false);
}


bool ItemTreeViewImpl::selectItem(Item* item, bool select)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        QModelIndex index = indexFromItem(itvItem);
        selectionModel()->select(index, (select ? QItemSelectionModel::Select : QItemSelectionModel::Deselect));
        return select;
    }
    return false;
}


void ItemTreeView::selectAllItems()
{
    impl->selectAll();
}


void ItemTreeView::clearSelection()
{
    impl->selectionModel()->clearSelection();
}


bool ItemTreeView::isItemChecked(ItemPtr item, int id)
{
    return impl->isItemChecked(item.get(), id);
}


bool ItemTreeViewImpl::isItemChecked(Item* item, int id)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        return (itvItem->checkState(id + 1) == Qt::Checked);
    }
    return false;
}
    

bool ItemTreeView::checkItem(ItemPtr item, bool checked, int id)
{
    return impl->checkItem(item.get(), checked, id);
}


bool ItemTreeViewImpl::checkItem(Item* item, bool checked, int id)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem){
        itvItem->setCheckState(id + 1, (checked ? Qt::Checked : Qt::Unchecked));
        return checked;
    }
    return false;
}


SignalProxy<void(const ItemList<>&)> ItemTreeView::sigSelectionChanged()
{
    return impl->sigSelectionChanged;
}


SignalProxy<void(const ItemList<>&)> ItemTreeView::sigSelectionOrTreeChanged()
{
    return impl->sigSelectionOrTreeChanged;
}


SignalProxy<void(Item* item, bool isChecked)> ItemTreeView::sigCheckToggled(int id)
{
    if(id < impl->checkColumns.size()){
        return impl->checkColumns[id]->sigCheckToggled;
    }
    return impl->sigCheckToggledForInvalidId; // or throw exception
}


SignalProxy<void(bool isChecked)> ItemTreeView::sigCheckToggled(Item* targetItem, int id)
{
    if(id < impl->checkColumns.size()){
        return impl->getOrCreateItvItem(targetItem)->getOrCreateSigCheckToggled(id);
    }
    return impl->sigCheckToggledForInvalidItem;
}


ItemList<>& ItemTreeView::allSelectedItems()
{
    return impl->selectedItemList;
}


void ItemTreeView::extractSelectedItemsOfSubTree(ItemPtr topItem, ItemList<>& io_items)
{
    topItem->traverse(boost::bind(&ItemTreeViewImpl::extractSelectedItemsOfSubTreeTraverse, impl, _1, &io_items));
}


bool ItemTreeViewImpl::extractSelectedItemsOfSubTreeTraverse(Item* item, ItemList<>* io_items)
{
    ItvItem* itvItem = getItvItem(item);
    if(itvItem && itvItem->isSelected()){
        io_items->push_back(item);
    }
    return false;
}


ItemList<>& ItemTreeView::allCheckedItems(int id)
{
    return impl->checkedItems(id);
}


ItemList<>& ItemTreeViewImpl::checkedItems(int id)
{
    if(id >= 0 && id < checkColumns.size()){
        CheckColumnPtr& cc = checkColumns[id];
        if(cc){
            if(cc->needToUpdateCheckedItemList){
                cc->checkedItemList.clear();
                extractCheckedItems(invisibleRootItem(), id + 1, cc->checkedItemList);
                cc->needToUpdateCheckedItemList = false;
            }
            return cc->checkedItemList;
        }
    }
    emptyItemList.clear();
    return emptyItemList;
}


void ItemTreeViewImpl::extractCheckedItems(QTreeWidgetItem* twItem, int column, ItemList<>& checkedItemList)
{
    ItvItem* itvItem = dynamic_cast<ItvItem*>(twItem);
    if(itvItem){
        if(itvItem->checkState(column) == Qt::Checked){
            checkedItemList.push_back(itvItem->item.get());
        }
    }
    int n = twItem->childCount();
    for(int i=0; i < n; ++i){
        extractCheckedItems(twItem->child(i), column, checkedItemList);
    }
}


void ItemTreeViewImpl::forEachTopItems(const ItemList<>& items, boost::function<void(Item*)> callback)
{
    set<Item*> itemSet;
    for(size_t i=0; i < items.size(); ++i){
        itemSet.insert(items.get(i));
    }

    for(size_t i=0; i < items.size(); ++i){
        Item* item = items.get(i);
        bool isChild = false;
        Item* parentItem = item->parentItem();
        while(parentItem){
            set<Item*>::iterator p = itemSet.find(parentItem);
            if(p != itemSet.end()){
                isChild = true;
                break;
            }
            parentItem = parentItem->parentItem();
        }
        if(!isChild){
            callback(item);
        }
    }
}


void ItemTreeViewImpl::cutSelectedItems()
{
    copiedItemList.clear();
    ItemList<> selected = selectedItemList;
    forEachTopItems(selected, boost::bind(&ItemTreeViewImpl::moveCutItemsToCopiedItemList, this, _1));
}


void ItemTreeView::cutSelectedItems()
{
    impl->cutSelectedItems();
}


void ItemTreeViewImpl::copySelectedItems()
{
    copiedItemList.clear();

    set<Item*> items;
    for(size_t i=0; i < selectedItemList.size(); ++i){
        items.insert(selectedItemList.get(i));
    }
    
    for(size_t i=0; i < selectedItemList.size(); ++i){
        Item* item = selectedItemList.get(i);
        bool isChild = false;
        Item* parentItem = item->parentItem();
        while(parentItem){
            set<Item*>::iterator p = items.find(parentItem);
            if(p != items.end()){
                isChild = true;
                break;
            }
            parentItem = parentItem->parentItem();
        }
        if(!isChild){
            ItemPtr duplicated = item->duplicate();
            if(!duplicated){
                //! \todo Put warning message. The item is probably a singleton item.
            } else {
                copiedItemList.push_back(duplicated);
                copySelectedItemsSub(item, duplicated, items);
            }
        }
    }
}


void ItemTreeViewImpl::copySelectedItemsSub(Item* item, ItemPtr& duplicated, set<Item*>& items)
{
    for(Item* childItem = item->childItem(); childItem; childItem = childItem->nextItem()){
        set<Item*>::iterator p = items.find(childItem);
        if(p != items.end()){
            ItemPtr duplicatedChild;
            if(childItem->isSubItem()){
                duplicatedChild = duplicated->findChildItem(childItem->name());
            } else {
                duplicatedChild = childItem->duplicate();
                if(duplicatedChild){
                    duplicated->addChildItem(duplicatedChild);
                }
            }
            if(duplicatedChild){
                copySelectedItemsSub(childItem, duplicatedChild, items);
            }
        }
    }
}


void ItemTreeViewImpl::copySelectedItemsWithChildren()
{
    copiedItemList.clear();
    forEachTopItems(selectedItemList, boost::bind(&ItemTreeViewImpl::addCopiedItemToCopiedItemList, this, _1));
}


void ItemTreeViewImpl::addCopiedItemToCopiedItemList(Item* item)
{
    ItemPtr duplicated = item->duplicateAll();
    if(duplicated){
        copiedItemList.push_back(duplicated);
    }
}


void ItemTreeViewImpl::pasteItems()
{
    if(!copiedItemList.empty()){
        ItemPtr parentItem;
        if(selectedItemList.empty()){
            parentItem = rootItem;
        } else if(selectedItemList.size() == 1){
            parentItem = selectedItemList.front();
        }
        if(parentItem){
            for(size_t i=0; i < copiedItemList.size(); ++i){
                ItemPtr org = copiedItemList[i];
                ItemPtr duplicated = org->duplicateAll();
                if(duplicated){
                    copiedItemList[i] = duplicated;
                }
                parentItem->addChildItem(org, true); // paste the original items
            }
        }
    }
}


void ItemTreeViewImpl::moveCutItemsToCopiedItemList(Item* item)
{
    if(!item->isSubItem()){
        copiedItemList.push_back(item);
        item->detachFromParentItem();
    }
}


void ItemTreeViewImpl::checkSelectedItems(bool on)
{
    for(size_t i=0; i < selectedItemList.size(); ++i){
        checkItem(selectedItemList[i].get(), on, 0);
    }
}


void ItemTreeViewImpl::toggleSelectedItemChecks()
{
    for(size_t i=0; i < selectedItemList.size(); ++i){
        Item* item = selectedItemList[i].get();
        checkItem(item, !isItemChecked(item, 0), 0);
    }
}


bool ItemTreeView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool ItemTreeViewImpl::storeState(Archive& archive)
{
    storeItemIds(archive, "selected", selectedItemList);
    storeItemIds(archive, "checked", checkedItems(0));
    storeExpandedItems(archive);
    return true;
}


void ItemTreeView::storeCheckColumnState(int id, Archive& archive)
{
    impl->storeItemIds(archive, "checked", impl->checkedItems(id));
}


void ItemTreeViewImpl::storeItemIds(Archive& archive, const char* key, const ItemList<>& items)
{
    ListingPtr idseq = new Listing();
    idseq->setFlowStyle(true);
    for(size_t i=0; i < items.size(); ++i){
        ValueNodePtr id = archive.getItemId(items[i]);
        if(id){
            idseq->append(id);
        }
    }
    if(!idseq->empty()){
        archive.insert(key, idseq);
    }
}


bool ItemTreeView::restoreState(const Archive& archive)
{
    archive.addPostProcess(boost::bind(&ItemTreeViewImpl::restoreState, impl, boost::ref(archive)));
    return true;
}


bool ItemTreeViewImpl::restoreState(const Archive& archive)
{
    restoreItemStates(archive, "checked", boost::bind(&ItemTreeView::checkItem, self, _1, true, 0));
    restoreItemStates(archive, "selected", boost::bind(&ItemTreeView::selectItem, self, _1, true));
    restoreExpandedItems(archive);
    return true;
}


bool ItemTreeView::restoreCheckColumnState(int id, const Archive& archive)
{
    return impl->restoreItemStates(archive, "checked", boost::bind(&ItemTreeView::checkItem, this, _1, true, id));
}


bool ItemTreeViewImpl::restoreItemStates
(const Archive& archive, const char* key, boost::function<void(ItemPtr)> stateChangeFunc)
{
    bool completed = false;
    const Listing& idseq = *archive.findListing(key);
    if(idseq.isValid()){
        completed = true;
        for(int i=0; i < idseq.size(); ++i){
            ValueNode* id = idseq.at(i);
            if(!id){
                completed = false;
            } else {
                Item* item = archive.findItem(id);
                if(item){
                    stateChangeFunc(item);
                } else {
                    completed = false;
                }
            }
        }
    }
    return completed;
}


void ItemTreeViewImpl::storeExpandedItems(Archive& archive)
{
    ListingPtr expanded = new Listing();
    expanded->setFlowStyle(true);
    storeExpandedItemsSub(invisibleRootItem(), archive, expanded);
    if(!expanded->empty()){
        archive.insert("expanded", expanded);
    }
}


void ItemTreeViewImpl::storeExpandedItemsSub(QTreeWidgetItem* parentTwItem, Archive& archive, ListingPtr& expanded)
{
    int n = parentTwItem->childCount();
    for(int i=0; i < n; ++i){
        ItvItem* itvItem = dynamic_cast<ItvItem*>(parentTwItem->child(i));
        if(itvItem){
            if(itvItem->isExpanded()){
                ValueNodePtr id = archive.getItemId(itvItem->item);
                if(id){
                    expanded->append(id);
                }
            }
            if(itvItem->childCount() > 0){
                storeExpandedItemsSub(itvItem, archive, expanded);
            }
        }
    }
}


void ItemTreeViewImpl::restoreExpandedItems(const Archive& archive)
{
    const Listing& expanded = *archive.findListing("expanded");
    if(expanded.isValid()){
        collapseAll();
        for(int i=0; i < expanded.size(); ++i){
            Item* item = archive.findItem(expanded.at(i));
            if(item){
                ItvItem* itvItem = getItvItem(item);
                if(itvItem){
                    itvItem->setExpanded(true);
                }
            }
        }
    }
}


void ItemTreeViewImpl::zoomFontSize(int pointSizeDiff)
{
    QFont font = TreeWidget::font();
    font.setPointSize(font.pointSize() + pointSizeDiff);
    setFont(font);
    fontPointSizeDiff += pointSizeDiff;
    AppConfig::archive()->openMapping("ItemTreeView")->write("fontZoom", fontPointSizeDiff);
}
