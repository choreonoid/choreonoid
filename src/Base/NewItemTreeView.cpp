#include "NewItemTreeView.h"
#include "ViewManager.h"
#include "ItemTreeWidget.h"
#include "RootItem.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class NewItemTreeView::Impl
{
public:
    NewItemTreeView* self;
    ItemTreeWidget* itemTreeWidget;

    Impl(NewItemTreeView* self);
};

}


void NewItemTreeView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<NewItemTreeView>(
        "NewItemTreeView", N_("Items"), ViewManager::SINGLE_OPTIONAL);
}


NewItemTreeView::NewItemTreeView()
{
    impl = new Impl(this);
}


NewItemTreeView::Impl::Impl(NewItemTreeView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::LEFT);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    itemTreeWidget = new ItemTreeWidget(RootItem::instance(), self);
    itemTreeWidget->addTopLevelItem(RootItem::instance());

    auto vbox = new QVBoxLayout;
    vbox->addWidget(itemTreeWidget);
    self->setLayout(vbox);
}


NewItemTreeView::~NewItemTreeView()
{
    delete impl;
}


bool NewItemTreeView::storeState(Archive& archive)
{
    return impl->itemTreeWidget->storeState(archive);
}


bool NewItemTreeView::restoreState(const Archive& archive)
{
    return impl->itemTreeWidget->restoreState(archive);
}
