#include "ItemTreeView.h"
#include "ItemManager.h"
#include "ViewManager.h"
#include "MenuManager.h"
#include "ItemTreeWidget.h"
#include "RootItem.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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
        "ItemTreeView", N_("Items"), ViewManager::SINGLE_DEFAULT);
}


ItemTreeView* ItemTreeView::instance()
{
    static ItemTreeView* instance_ = ViewManager::getOrCreateView<ItemTreeView>();
    return instance_;
}


ItemTreeView::ItemTreeView()
{
    impl = new Impl(this);
}


ItemTreeView::Impl::Impl(ItemTreeView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::LEFT);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    itemTreeWidget = new ItemTreeWidget(self);

    auto vbox = new QVBoxLayout;
    vbox->addWidget(itemTreeWidget);
    self->setLayout(vbox);

    itemTreeWidget->customizeContextMenu<Item>(
        [&](Item* item, MenuManager& menuManager, ItemFunctionDispatcher){
            onContextMenuRequested(item, menuManager); });
}


ItemTreeView::~ItemTreeView()
{
    delete impl;
}


ItemTreeWidget* ItemTreeView::itemTreeWidget()
{
    return impl->itemTreeWidget;
}


void ItemTreeView::Impl::onContextMenuRequested(Item* item, MenuManager& menuManager)
{
    menuManager.addItem(_("Cut"))->sigTriggered().connect(
        [&](){ itemTreeWidget->cutSelectedItems(); });
    
    menuManager.addItem(_("Copy (single)"))->sigTriggered().connect(
        [&](){ itemTreeWidget->copySelectedItems(); });
    
    menuManager.addItem(_("Copy (sub tree)"))->sigTriggered().connect(
        [&](){ itemTreeWidget->copySelectedItemsWithSubTrees(); });
    
    menuManager.addItem(_("Paste"))->sigTriggered().connect(
        [&](){ itemTreeWidget->pasteItems(); });

    menuManager.addSeparator();
    
    menuManager.addItem(_("Check"))->sigTriggered().connect(
        [&](){ itemTreeWidget->setSelectedItemsChecked(true); });
    
    menuManager.addItem(_("Uncheck"))->sigTriggered().connect(
        [&](){ itemTreeWidget->setSelectedItemsChecked(false); });
    
    menuManager.addItem(_("Toggle checks"))->sigTriggered().connect(
        [&](){ itemTreeWidget->toggleSelectedItemChecks(); });

    menuManager.addSeparator();
    
    menuManager.addItem(_("Reload"))->sigTriggered().connect(
        [&](){ ItemManager::reloadItems(itemTreeWidget->selectedItems()); });

    menuManager.addSeparator();
    
    menuManager.addItem(_("Select all"))->sigTriggered().connect(
        [=](){ itemTreeWidget->selectAllItems(); });
    
    menuManager.addItem(_("Clear selection"))->sigTriggered().connect(
        [=](){ itemTreeWidget->clearSelection(); });
}


bool ItemTreeView::storeState(Archive& archive)
{
    return impl->itemTreeWidget->storeState(archive);
}


bool ItemTreeView::restoreState(const Archive& archive)
{
    if(impl->itemTreeWidget->restoreState(archive)){
        archive.addPostProcess([&](){ impl->restoreState(archive); });
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
        archive, "selected", [&](Item* item){ item->setSelected(true); });
    restoreItemStates(
        archive, "checked", [&](Item* item){ item->setChecked(true); });
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
