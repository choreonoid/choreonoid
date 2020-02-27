#include "ItemPropertyView.h"
#include "ItemPropertyWidget.h"
#include "TargetItemPicker.h"
#include "ViewManager.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class ItemPropertyView::Impl : public ItemPropertyWidget
{
public:
    TargetItemPicker<Item> targetItemPicker;

    Impl(ItemPropertyView* self);
};

}


void ItemPropertyView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<ItemPropertyView>(
        "ItemPropertyView", N_("Property"), ViewManager::SINGLE_DEFAULT);
}


ItemPropertyView::ItemPropertyView()
{
    impl = new Impl(this);

    setDefaultLayoutArea(View::LEFT_BOTTOM);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(impl);
    setLayout(vbox);
}


ItemPropertyView::Impl::Impl(ItemPropertyView* self)
    : ItemPropertyWidget(self),
      targetItemPicker(self)
{
    setPropertyFunction<Item>(
        [](Item* item, PutPropertyFunction& putProperty){
            item->putProperties(putProperty);
        });
    
    targetItemPicker.sigTargetItemSpecified().connect(
        [&](Item* item){ setCurrentItem(item); });
}


ItemPropertyView::~ItemPropertyView()
{
    delete impl;
}


void ItemPropertyView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->setOperationMenu(menuManager);
}
