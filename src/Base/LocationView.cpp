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
#include <cnoid/ConnectionSet>
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
    ScopedConnectionSet targetItemConnections;
    QLabel caption;
    CheckBox lockCheck;
    PositionWidget* positionWidget;
    vector<CoordinateInfoPtr> coordinates;
    QLabel coordinateLabel;
    ComboBox coordinateCombo;
    int defaultCoordIndex;
        
    Impl(LocationView* self);
    void setTargetItem(Item* item);
    void setLocked(bool on);
    void onLockCheckToggled(bool on);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void updatePositionWidgetWithTargetLocation();
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
    lockCheck.sigToggled().connect(
        [&](bool on){ onLockCheckToggled(on); });
    hbox->addWidget(&lockCheck);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    coordinateLabel.setText(_("Coord :"));
    hbox->addWidget(&coordinateLabel, 0);
    coordinateCombo.sigAboutToShowPopup().connect(
        [&](){ updateBaseCoordinateSystems(); });
    coordinateCombo.sigCurrentIndexChanged().connect(
        [&](int index){ updatePositionWidgetWithTargetLocation(); });
    hbox->addWidget(&coordinateCombo, 1);
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);
    positionWidget->setPositionCallback(
        [&](const Position& T){ return setInputPositionToTargetItem(T); });
    vbox->addWidget(positionWidget);

    vbox->addStretch();

    targetItem = nullptr;
    targetItemPicker.setTargetInterface<LocatableItem>();
    targetItemPicker.sigTargetItemChanged().connect(
        [&](Item* item){ setTargetItem(item); });
    setTargetItem(nullptr);

    // Use the parent coordinate system by default
    defaultCoordIndex = 1;
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
    targetItemConnections.disconnect();
    targetItem = dynamic_cast<LocatableItem*>(item);
    
    if(!targetItem){
        caption.setText("-----");
        setLocked(false);
        positionWidget->updatePosition(Position::Identity());
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();
        
    } else {
        caption.setText(item->name().c_str());
        setLocked(!targetItem->getLocationEditable());

        targetItemConnections.add(
            targetItem->sigLocationChanged().connect(
                [this](){ updatePositionWidgetWithTargetLocation(); }));
        
        targetItemConnections.add(
            targetItem->sigLocationEditableToggled().connect(
                [this](bool on){ setLocked(!on); }));
            
        updateBaseCoordinateSystems();
        updatePositionWidgetWithTargetLocation();
    }

    lockCheck.blockSignals(false);
}


void LocationView::Impl::setLocked(bool on)
{
    lockCheck.blockSignals(true);
    lockCheck.setChecked(on);
    positionWidget->setEditable(!on);
    lockCheck.blockSignals(false);
}
    

void LocationView::Impl::onLockCheckToggled(bool on)
{
    if(targetItem){
        targetItemConnections.block();
        targetItem->setLocationEditable(!on);
        positionWidget->setEditable(!on);
        targetItemConnections.unblock();
    }
}


void LocationView::Impl::clearBaseCoordinateSystems()
{
    coordinateCombo.blockSignals(true);
    coordinates.clear();
    coordinateCombo.clear();
    coordinateLabel.setEnabled(false);
    coordinateCombo.setEnabled(false);
    coordinateCombo.blockSignals(false);
}


void LocationView::Impl::updateBaseCoordinateSystems()
{
    if(!targetItem){
        return;
    }
    
    int currentIndex = coordinateCombo.currentIndex();

    clearBaseCoordinateSystems();

    auto worldCoord = new CoordinateInfo(_("World"));
    worldCoord->T.setIdentity();
    coordinates.push_back(worldCoord);

    if(targetItem){
        CoordinateInfo* parentCoord;
        LocatableItem* parentLocatable = targetItem->getParentLocatableItem();
        if(!parentLocatable){
            parentCoord = new CoordinateInfo(_("Parent ( World )"));
        } else {
            ItemPtr parentItem = parentLocatable->getCorrespondingItem();
            parentCoord = new CoordinateInfo(
                format(_("Parent ( {} )"), parentLocatable->getLocationName()));
            // parentItem is captured to keep parentLocatable alive until the function is disposed
            parentCoord->parentPositionFunc =
                [parentItem, parentLocatable](){ return parentLocatable->getLocation(); };
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

    coordinateLabel.setEnabled(true);
    coordinateCombo.setEnabled(true);
    coordinateCombo.blockSignals(true);
    for(auto& coord : coordinates){
        coordinateCombo.addItem(coord->name.c_str());
    }

    if(currentIndex < 0){
        currentIndex = defaultCoordIndex;
    }
    coordinateCombo.setCurrentIndex(currentIndex);
    coordinateCombo.blockSignals(false);
}


void LocationView::Impl::updatePositionWidgetWithTargetLocation()
{
    if(!targetItem){
        return;
    }
    auto& coord = coordinates[coordinateCombo.currentIndex()];
    Position T_world = targetItem->getLocation();
    Position T;
    if(coord->parentPositionFunc){
        Position T_base = coord->parentPositionFunc() * coord->T;
        T = T_base.inverse(Eigen::Isometry) * T_world;
    } else {
        T = coord->T.inverse(Eigen::Isometry) * T_world;
    }
    positionWidget->updatePosition(T);
}
        

bool LocationView::Impl::setInputPositionToTargetItem(const Position& T)
{
    if(targetItem){
        auto& coord = coordinates[coordinateCombo.currentIndex()];
        Position T_world;
        if(coord->parentPositionFunc){
            Position T_base = coord->parentPositionFunc() * coord->T;
            T_world = T_base * T;
        } else {
            T_world = coord->T * T;
        }
        targetItemConnections.block();
        targetItem->setLocation(T_world);
        targetItemConnections.unblock();
        return true;
    }
    return false;
}


bool LocationView::storeState(Archive& archive)
{
    impl->positionWidget->storeState(archive);
    int coordIndex = impl->coordinateCombo.currentIndex();
    if(coordIndex >= 0){
        archive.write("current_coord_index", coordIndex);
    }
    return true;
}


bool LocationView::restoreState(const Archive& archive)
{
    impl->positionWidget->restoreState(archive);

    if(archive.read("current_coord_index", impl->defaultCoordIndex)){
        if(impl->coordinateCombo.count() > impl->defaultCoordIndex){
            impl->coordinateCombo.setCurrentIndex(impl->defaultCoordIndex);
        }
    }
    return true;
}
