#include "ItemPropertyView.h"
#include "ItemPropertyWidget.h"
#include "TargetItemPicker.h"
#include "ViewManager.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

ItemPropertyView* instance_ = nullptr;

}

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
        N_("ItemPropertyView"), N_("Property"), ViewManager::Permanent);
}


ItemPropertyView* ItemPropertyView::instance()
{
    if(!instance_){
        instance_ = ViewManager::getOrCreateView<ItemPropertyView>();
    }
    return instance_;
}


ItemPropertyView::ItemPropertyView()
{
    instance_ = this;
    impl = new Impl(this);

    setDefaultLayoutArea(BottomLeftArea);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->addWidget(impl);
    setLayout(vbox);
}


ItemPropertyView::Impl::Impl(ItemPropertyView* self)
    : ItemPropertyWidget(self),
      targetItemPicker(self)
{
    setPropertyFunction<Item>(
        [this](Item* item, PutPropertyFunction& putProperty){
            if(hasAdditionalPropertyFunctionFor(item)){
                item->putProperties(putProperty,
                    [this](Item* item, PutPropertyFunction& /*putProperty*/){
                        dispatchAdditionalPropertyFunctions(item);
                    });
            } else {
                item->putProperties(putProperty);
            }
        });

    targetItemPicker.sigTargetItemSpecified().connect(
        [&](Item* item){ setCurrentItem(item); });
}


ItemPropertyView::~ItemPropertyView()
{
    if(instance_ == this){
        instance_ = nullptr;
    }
    delete impl;
}


void ItemPropertyView::addPropertyFunction_(
    const std::type_info& type,
    std::function<void(Item* item, PutPropertyFunction& putProperty)> func)
{
    impl->addPropertyFunction_(type, func);
}


void ItemPropertyView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->setOperationMenu(menuManager);
}
