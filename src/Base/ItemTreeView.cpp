#include "ItemTreeView.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "ItemTreeWidget.h"
#include "RootItem.h"
#include "Archive.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

ItemTreeView* instance_ = nullptr;

}

namespace cnoid {

class ItemTreeView::Impl
{
public:
    ItemTreeView* self;
    ItemTreeWidget* itemTreeWidget;

    Impl(ItemTreeView* self);
    void onContextMenuRequested(Item* item, MenuManager& menuManager);
    void restoreState(const Archive& archive);
    void restoreItemStates(
        const Archive& archive, const char* key, std::function<void(Item*)> stateChangeFunc);
};

}


void ItemTreeView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ItemTreeView>(
        N_("ItemTreeView"), N_("Items"), ViewManager::Permanent);
}


ItemTreeView* ItemTreeView::instance()
{
    if(!instance_){
        instance_ = ViewManager::getOrCreateView<ItemTreeView>();
    }
    return instance_;
}


ItemTreeView* ItemTreeView::findInstance()
{
    if(!instance_){
        instance_ = ViewManager::findView<ItemTreeView>();
    }
    return instance_;
}


ItemTreeView::ItemTreeView()
{
    impl = new Impl(this);
}


ItemTreeView::Impl::Impl(ItemTreeView* self)
    : self(self)
{
    self->setDefaultLayoutArea(TopLeftArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    itemTreeWidget = new ItemTreeWidget(self);
    itemTreeWidget->setItemNameEditByDoubleClickEnabled(false);
    itemTreeWidget->setRootItem(RootItem::instance(), false);

    auto vbox = new QVBoxLayout;
    vbox->addWidget(itemTreeWidget);
    self->setLayout(vbox);

    itemTreeWidget->customizeRootContextMenu(
        [this](MenuManager& menu){ onContextMenuRequested(nullptr, menu); });
    
    itemTreeWidget->customizeContextMenu<Item>(
        [this](Item* item, MenuManager& menu, ItemFunctionDispatcher){
            onContextMenuRequested(item, menu); });
}


ItemTreeView::~ItemTreeView()
{
    delete impl;
}


ItemTreeWidget* ItemTreeView::itemTreeWidget()
{
    return impl->itemTreeWidget;
}


void ItemTreeView::setExpanded(Item* item, bool on)
{
    impl->itemTreeWidget->setExpanded(item, on);
}


void ItemTreeView::Impl::onContextMenuRequested(Item* item, MenuManager& menu)
{
    auto rename = menu.addItem(_("Rename"));
    menu.addSeparator();
    
    auto cut = menu.addItem(_("Cut"));
    auto copy1 = menu.addItem(_("Copy (single)"));
    auto copy2 = menu.addItem(_("Copy (sub tree)"));
    auto paste = menu.addItem(_("Paste"));
    menu.addSeparator();
    
    auto check = menu.addItem(_("Check"));
    auto uncheck = menu.addItem(_("Uncheck"));
    auto toggleCheck = menu.addItem(_("Toggle checks"));
    menu.addSeparator();

    auto reload = menu.addItem(_("Reload"));
    auto saveAs = menu.addItem(_("Save as"));
    menu.addSeparator();
    
    auto selectAll = menu.addItem(_("Select all"));
    auto clearSelection = menu.addItem(_("Clear selection"));

    bool isContinuousUpdateStateSubTree = item->isContinuousUpdateStateSubTree();

    if(!item || isContinuousUpdateStateSubTree){
        rename->setEnabled(false);
        cut->setEnabled(false);
        copy1->setEnabled(false);
        copy2->setEnabled(false);
        paste->setEnabled(false);
        check->setEnabled(false);
        uncheck->setEnabled(false);
        toggleCheck->setEnabled(false);
        reload->setEnabled(false);
        saveAs->setEnabled(false);
        clearSelection->setEnabled(false);

    } else {
        if(item->isSubItem() || item->hasAttribute(Item::Attached)){
            rename->setEnabled(false);
        } else {
            rename->sigTriggered().connect(
                [this, item](){ itemTreeWidget->editItemName(item); });
        }
            
        if(itemTreeWidget->checkCuttable(item)){
            cut->sigTriggered().connect(
                [this](){ itemTreeWidget->cutSelectedItems(); });
        } else {
            cut->setEnabled(false);
        }
        if(itemTreeWidget->checkCopiable(item)){
            copy1->sigTriggered().connect(
                [this](){ itemTreeWidget->copySelectedItems(); });
            copy2->sigTriggered().connect(
                [this](){ itemTreeWidget->copySelectedItemsWithSubTrees(); });
        } else {
            copy1->setEnabled(false);
            copy2->setEnabled(false);
        }

        if(!item->hasAttribute(Item::Reloadable)){
            reload->setEnabled(false);
        } else {
            reload->sigTriggered().connect(
                [this](){
                    for(auto& item : itemTreeWidget->getSelectedItems()){
                        item->reload();
                    }
                });
        }
        if(!item->isFileSavable()){
            saveAs->setEnabled(false);
        } else {
            saveAs->sigTriggered().connect(
                [this, item](){ item->saveWithFileDialog(); });
        }

        clearSelection->sigTriggered().connect(
            [this](){ itemTreeWidget->clearSelection(); });
    }

    if(item){
        clearSelection->setEnabled(true);
        check->sigTriggered().connect(
            [this](){ itemTreeWidget->setSelectedItemsChecked(true); });
        uncheck->sigTriggered().connect(
            [this](){ itemTreeWidget->setSelectedItemsChecked(false); });
        toggleCheck->sigTriggered().connect(
            [this](){ itemTreeWidget->toggleSelectedItemChecks(); });
    }

    if(itemTreeWidget->checkPastable(item) && !(item && isContinuousUpdateStateSubTree)){
        paste->setEnabled(true);
        paste->sigTriggered().connect(
            [this](){ itemTreeWidget->pasteItems(); });
    } else {
        paste->setEnabled(false);
    }

    selectAll->sigTriggered().connect(
        [this](){ itemTreeWidget->selectAllItems(); });
}


bool ItemTreeView::storeState(Archive& archive)
{
    return impl->itemTreeWidget->storeState(archive);
}


bool ItemTreeView::restoreState(const Archive& archive)
{
    if(impl->itemTreeWidget->restoreState(archive)){
        archive.addPostProcess([this, &archive](){ impl->restoreState(archive); });
        return true;
    }
    return false;
}


void ItemTreeView::Impl::restoreState(const Archive& archive)
{
    /**
       The selection state and check state of each item are usually
       restored from the archive for the item object, but the old format
       stores the states in the archive for ItemTreeView. To support
       the old format, the states are restored from the archive for the
       ItemTreeView when the corresponding data is stored in it.
    */
    restoreItemStates(
        archive, "selected", [this](Item* item){ item->setSelected(true); });
    restoreItemStates(
        archive, "checked", [this](Item* item){ item->setChecked(true); });
}


void ItemTreeView::Impl::restoreItemStates
(const Archive& archive, const char* key, std::function<void(Item*)> stateChangeFunc)
{
    const Listing& idseq = *archive.findListing(key);
    if(idseq.isValid()){
        for(int i=0; i < idseq.size(); ++i){
            if(auto id = idseq.at(i)){
                Item* item = archive.findItem(id);
                if(item){
                    stateChangeFunc(item);
                }
            }
        }
    }
}
