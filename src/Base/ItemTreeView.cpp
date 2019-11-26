#include "ItemTreeView.h"
#include "ViewManager.h"
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

    itemTreeWidget = new ItemTreeWidget(RootItem::instance(), self);

    auto vbox = new QVBoxLayout;
    vbox->addWidget(itemTreeWidget);
    self->setLayout(vbox);
}


ItemTreeView::~ItemTreeView()
{
    delete impl;
}


void ItemTreeView::setExpanded(Item* item, bool on)
{
    impl->itemTreeWidget->setExpanded(item, on);
}


bool ItemTreeView::storeState(Archive& archive)
{
    return impl->itemTreeWidget->storeState(archive);
}


bool ItemTreeView::restoreState(const Archive& archive)
{
    archive.addPostProcess([&](){ impl->restoreState(archive); });
    return true;
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
    
    itemTreeWidget->restoreState(archive);
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
