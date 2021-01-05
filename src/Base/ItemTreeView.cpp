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

    itemTreeWidget->customizeRootContextMenu(
        [&](MenuManager& menu){ onContextMenuRequested(nullptr, menu); });
    
    itemTreeWidget->customizeContextMenu<Item>(
        [&](Item* item, MenuManager& menu, ItemFunctionDispatcher){
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

    menu.addSeparator();
    
    auto selectAll = menu.addItem(_("Select all"));
    auto clearSelection = menu.addItem(_("Clear selection"));

    if(item){
        cut->sigTriggered().connect(
            [&](){ itemTreeWidget->cutSelectedItems(); });
        copy1->sigTriggered().connect(
            [&](){ itemTreeWidget->copySelectedItems(); });
        copy2->sigTriggered().connect(
            [&](){ itemTreeWidget->copySelectedItemsWithSubTrees(); });
        check->sigTriggered().connect(
            [&](){ itemTreeWidget->setSelectedItemsChecked(true); });
        uncheck->sigTriggered().connect(
            [&](){ itemTreeWidget->setSelectedItemsChecked(false); });
        toggleCheck->sigTriggered().connect(
            [&](){ itemTreeWidget->toggleSelectedItemChecks(); });

        reload->sigTriggered().connect(
            [&](){
                for(auto& item : itemTreeWidget->getSelectedItems()){
                    item->reload();
                }
            });

        clearSelection->sigTriggered().connect(
            [=](){ itemTreeWidget->clearSelection(); });
    } else {
        cut->setEnabled(false);
        copy1->setEnabled(false);
        copy2->setEnabled(false);
        check->setEnabled(false);
        uncheck->setEnabled(false);
        toggleCheck->setEnabled(false);
        reload->setEnabled(false);
        clearSelection->setEnabled(false);
    }

    paste->sigTriggered().connect(
        [&](){ itemTreeWidget->pasteItems(); });
    selectAll->sigTriggered().connect(
        [=](){ itemTreeWidget->selectAllItems(); });
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
