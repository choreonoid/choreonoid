#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "TargetItemPicker.h"
#include "LocatableItem.h"
#include "CoordinateFrameListItem.h"
#include "RootItem.h"
#include "CheckBox.h"
#include "ComboBox.h"
#include <cnoid/CoordinateFrameList>
#include <QBoxLayout>
#include <QLabel>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

struct CoordinateInfo : public Referenced
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    string name;
    Position T;
    std::function<Position()> parentPositionFunc;
    bool isLocal;
    CoordinateInfo(const std::string& name, const Position& T, const std::function<Position()>& func)
        : name(name), T(T), parentPositionFunc(func), isLocal(false) { }
    CoordinateInfo(const std::string& name, const Position& T)
        : CoordinateInfo(name, T, nullptr) { }
    CoordinateInfo(const std::string& name)
        : CoordinateInfo(name, Position::Identity(), nullptr) { }
};
typedef ref_ptr<CoordinateInfo> CoordinateInfoPtr;

}

namespace cnoid {

class LocationView::Impl
{
public:
    LocationView* self;
    LocatableItem* targetItem;
    TargetItemPicker<Item> targetItemPicker;
    ScopedConnection targetItemConnection;
    QLabel caption;
    CheckBox lockCheck;
    PositionWidget* positionWidget;
    vector<CoordinateInfoPtr> coordinates;
    ComboBox coordinateCombo;
        
    Impl(LocationView* self);
    void setTargetItem(Item* item);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
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

    auto hbox = new QHBoxLayout;
    caption.setStyleSheet("font-weight: bold");
    hbox->addWidget(&caption, 1);
    lockCheck.setText(_("Lock"));
    hbox->addWidget(&lockCheck);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coord :")), 0);
    coordinateCombo.sigAboutToShowPopup().connect(
        [&](){ updateBaseCoordinateSystems(); });
    hbox->addWidget(&coordinateCombo, 1);
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);
    //positionWidget->setCaptionVisible(true);
    //positionWidget->setBuiltinCoordinateSystemComboEnabled(true);
    positionWidget->setPositionCallback(
        [&](const Position& T){ return setInputPositionToTargetItem(T); });
    vbox->addWidget(positionWidget);

    vbox->addStretch();

    targetItem = nullptr;
    targetItemPicker.setTargetInterface<LocatableItem>();
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
    targetItem = dynamic_cast<LocatableItem*>(item);
    
    if(!targetItem){
        targetItemConnection.disconnect();
        caption.setText("-----");
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();
    } else {
        caption.setText(item->name().c_str());
        positionWidget->updatePosition(targetItem->getLocation());
        positionWidget->setEditable(targetItem->isLocationEditable());
        targetItemConnection =
            targetItem->sigLocationChanged().connect(
                [this](){
                    positionWidget->updatePosition(targetItem->getLocation()); });
        updateBaseCoordinateSystems();
    }
}


void LocationView::Impl::clearBaseCoordinateSystems()
{
    coordinates.clear();
    coordinateCombo.clear();
}


void LocationView::Impl::updateBaseCoordinateSystems()
{
    int index = coordinateCombo.currentIndex();
    if(index < 0){
        index = 0;
    }

    clearBaseCoordinateSystems();

    auto worldCoord = new CoordinateInfo(_("World"));
    worldCoord->T.setIdentity();
    coordinates.push_back(worldCoord);

    if(targetItem){
        CoordinateInfo* parentCoord;
        if(!targetItem->hasParentLocation()){
            parentCoord = new CoordinateInfo(_("Parent ( World )"));
        } else {
            string parentName = targetItem->getParentLocationName();
            parentCoord = new CoordinateInfo(format(_("Parent ( {} )"), parentName));
            parentCoord->parentPositionFunc =
                [&](){ return targetItem->getParentLocation(); };
        }
        coordinates.push_back(parentCoord);
        
        auto localCoord = new CoordinateInfo(_("Local"), targetItem->getLocation());
        localCoord->isLocal = true;
        coordinates.push_back(localCoord);

        //! \todo Use WorldItem as the root of the item search tree
        auto frameListItems = RootItem::instance()->descendantItems<CoordinateFrameListItem>();
        string basename;
        for(auto& frameListItem : frameListItems){
            basename.clear();
            function<Position()> parentPositionFunc;
            if(auto parentItem = frameListItem->getParentLocatableItem()){
                basename += dynamic_cast<Item*>(parentItem)->name() + " : ";
                parentPositionFunc =
                    [parentItem](){ return parentItem->getLocation(); };
            }
            basename += frameListItem->name();
            auto frames = frameListItem->frameList();
            int n = frames->numFrames();
            for(int i=0; i < n; ++i){
                string name(basename);
                auto frame = frames->frameAt(i);
                name += frame->id().label();
                auto coord = new CoordinateInfo(name, frame->T(), parentPositionFunc);
                coordinates.push_back(coord);
            }
        }
    }
            
    for(auto& coord : coordinates){
        coordinateCombo.addItem(coord->name.c_str());
    }
    coordinateCombo.setCurrentIndex(index);
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
