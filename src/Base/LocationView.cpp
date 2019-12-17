#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "TargetItemPicker.h"
#include "PlaceableItem.h"
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LocationView::Impl : public PositionWidget
{
public:
    PlaceableItem* targetItem;
    TargetItemPicker<Item> targetItemPicker;
    ScopedConnection connection;
    
    Impl(QWidget* parent);
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
    setDefaultLayoutArea(View::CENTER);
    impl = new Impl(this);
    auto vbox = new QVBoxLayout;
    vbox->addWidget(impl);
    setLayout(vbox);
}


LocationView::Impl::Impl(QWidget* parent)
    : PositionWidget(parent)
{
    targetItem = nullptr;
    
    targetItemPicker.setTargetInterface<PlaceableItem>();
    targetItemPicker.sigTargetItemChanged().connect(
        [&](Item* item){ setTargetItem(item); });

    setPositionCallback(
        [&](const Position& T){ return setInputPositionToTargetItem(T); });
}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->setOptionMenu(menuManager);
}


void LocationView::Impl::setTargetItem(Item* item)
{
    targetItem = dynamic_cast<PlaceableItem*>(item);
    
    if(!targetItem){
        connection.disconnect();
    } else {
        updatePosition(targetItem->getLocation());
        connection =
            targetItem->sigLocationChanged().connect(
                [this](){
                    updatePosition(targetItem->getLocation()); });
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
    impl->storeState(archive);
    return true;
}


bool LocationView::restoreState(const Archive& archive)
{
    impl->restoreState(archive);
    return true;
}
