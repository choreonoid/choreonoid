#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "TargetItemPicker.h"
#include "PlaceableItem.h"
#include <QBoxLayout>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class LocationView::Impl
{
public:
    LocationView* self;
    PlaceableItem* targetItem;
    TargetItemPicker<Item> targetItemPicker;
    ScopedConnection connection;
    PositionWidget* positionWidget;
    
    Impl(LocationView* self);
    void setTargetItem(Item* item);
    bool setInputPositionToTargetItem(const Position& T);
};

}


void LocationView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LocationView>(
        "LocationView", N_("Location"), ViewManager::SINGLE_OPTIONAL);
}


LocationView::LocationView()
{
    impl = new Impl(this);
}


LocationView::Impl::Impl(LocationView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox, 0.5);

    positionWidget = new PositionWidget(self);
    positionWidget->setCaptionVisible(true);
    positionWidget->setPositionCallback(
        [&](const Position& T){ return setInputPositionToTargetItem(T); });
    vbox->addWidget(positionWidget);
    vbox->addStretch();

    targetItem = nullptr;
    targetItemPicker.setTargetInterface<PlaceableItem>();
    targetItemPicker.sigTargetItemChanged().connect(
        [&](Item* item){ setTargetItem(item); });
    setTargetItem(nullptr);
}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenu(menuManager);
}


void LocationView::Impl::setTargetItem(Item* item)
{
    targetItem = dynamic_cast<PlaceableItem*>(item);
    
    if(!targetItem){
        connection.disconnect();
        positionWidget->setCaption("-----");
        positionWidget->setEditable(false);
    } else {
        positionWidget->setCaption(item->name());
        positionWidget->updatePosition(targetItem->getLocation());
        positionWidget->setEditable(targetItem->isLocationEditable());
        connection =
            targetItem->sigLocationChanged().connect(
                [this](){
                    positionWidget->updatePosition(targetItem->getLocation()); });
    }
}


bool LocationView::Impl::setInputPositionToTargetItem(const Position& T)
{
    if(targetItem){
        targetItem->setLocation(T);
        return true;
    }
    return false;
}


bool LocationView::storeState(Archive& archive)
{
    impl->positionWidget->storeState(archive);
    return true;
}


bool LocationView::restoreState(const Archive& archive)
{
    impl->positionWidget->restoreState(archive);
    return true;
}
