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

enum CoordinateIndexId {
    WorldCoordIndex = -2,
    ParentCoordIndex = -3,
    LocalCoordIndex = -4
};

struct CoordinateInfo : public Referenced
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    string name;
    int index;
    Position T;
    std::function<Position()> parentPositionFunc;
    Matrix3 R0; // the initial rotation of the local coordinate system
    bool isLocal;
    CoordinateInfo(const std::string& name, int index, const Position& T, const std::function<Position()>& func)
        : name(name), index(index), T(T), parentPositionFunc(func), isLocal(false) { }
    CoordinateInfo(const std::string& name, int index, const Position& T)
        : CoordinateInfo(name, index, T, nullptr) { }
    CoordinateInfo(const std::string& name, int index)
        : CoordinateInfo(name, index, Position::Identity(), nullptr) { }
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
    int lastCoordIndex;
        
    Impl(LocationView* self);
    void setTargetItem(Item* item);
    void setLocked(bool on);
    void onLockCheckToggled(bool on);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void updatePositionWidgetWithTargetLocation();
    bool setInputPositionToTargetItem(const Position& T);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
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
    coordinateCombo.sigActivated().connect(
        [&](int){ updatePositionWidgetWithTargetLocation(); });
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

    lastCoordIndex = ParentCoordIndex;
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
        positionWidget->setPosition(Position::Identity());
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
    
    clearBaseCoordinateSystems();
    int defaultComboIndex = 0;

    if(!targetItem->prefersLocalLocation()){
        auto worldCoord = new CoordinateInfo(_("World"), WorldCoordIndex);
        worldCoord->T.setIdentity();
        coordinates.push_back(worldCoord);
    }

    if(targetItem){
        CoordinateInfo* parentCoord;
        LocatableItem* parentLocatable = targetItem->getParentLocatableItem();
        if(parentLocatable){
            ItemPtr parentItem = parentLocatable->getCorrespondingItem();
            parentCoord = new CoordinateInfo(
                format(_("Parent ( {} )"), parentLocatable->getLocationName()), ParentCoordIndex);
            // parentItem is captured to keep parentLocatable alive until the function is disposed
            parentCoord->parentPositionFunc =
                [parentItem, parentLocatable](){ return parentLocatable->getLocation(); };
            defaultComboIndex = coordinates.size();
            coordinates.push_back(parentCoord);
        }

        Position T = targetItem->getLocation();
        auto localCoord = new CoordinateInfo(_("Local"), LocalCoordIndex, T);
        localCoord->isLocal = true;
        localCoord->R0 = T.linear();
        coordinates.push_back(localCoord);

        if(!targetItem->prefersLocalLocation()){
            //! \todo Use WorldItem as the root of the item search tree
            auto frameListItems =
                RootItem::instance()->descendantItems<CoordinateFrameListItem>();
            string basename;
            for(auto& frameListItem : frameListItems){
                basename.clear();
                function<Position()> parentPositionFunc;
                auto parentItem = frameListItem->getParentLocatableItem();
                if(!parentItem ||
                   parentItem->getCorrespondingItem() == targetItem->getCorrespondingItem()){
                    continue;
                }
                basename += parentItem->getLocationName() + " : ";
                parentPositionFunc =
                    [parentItem](){ return parentItem->getLocation(); };
                basename += frameListItem->name() + " ";
                auto frames = frameListItem->frameList();
                int n = frames->numFrames();
                for(int i=0; i < n; ++i){
                    string name(basename);
                    auto frame = frames->frameAt(i);
                    name += frame->id().label();
                    auto coord = new CoordinateInfo(name, coordinates.size(), frame->T(), parentPositionFunc);
                    coordinates.push_back(coord);
                }
            }
        }
    }

    coordinateLabel.setEnabled(true);
    coordinateCombo.setEnabled(true);
    coordinateCombo.blockSignals(true);
    int comboIndex = -1;
    for(auto& coord : coordinates){
        if(coord->index == lastCoordIndex){
            comboIndex = coordinateCombo.count();
        }
        coordinateCombo.addItem(coord->name.c_str());
    }

    if(comboIndex < 0){
        comboIndex = defaultComboIndex;
    }
    coordinateCombo.setCurrentIndex(comboIndex);
    lastCoordIndex = coordinates[comboIndex]->index;
    coordinateCombo.blockSignals(false);
}


void LocationView::Impl::updatePositionWidgetWithTargetLocation()
{
    if(!targetItem){
        return;
    }
    auto& coord = coordinates[coordinateCombo.currentIndex()];
    lastCoordIndex = coord->index;
    Position T_world = targetItem->getLocation();

    Position T;
    if(!coord->isLocal){
        if(coord->parentPositionFunc){
            Position T_base = coord->parentPositionFunc() * coord->T;
            T = T_base.inverse(Eigen::Isometry) * T_world;
        } else {
            T = coord->T.inverse(Eigen::Isometry) * T_world;
        }
        
    } else {
        Matrix3 R_prev = coord->T.linear();
        Matrix3 R_current = T_world.linear();
        if(!R_current.isApprox(R_prev)){
            coord->T = T_world;
        }
        T = coord->T.inverse(Eigen::Isometry) * T_world;
        T.linear() = coord->R0.transpose() * T_world.linear();
    }
    positionWidget->setPosition(T);
}
        

bool LocationView::Impl::setInputPositionToTargetItem(const Position& T)
{
    if(targetItem){
        auto& coord = coordinates[coordinateCombo.currentIndex()];
        Position T_world;
        if(!coord->isLocal){
            if(coord->parentPositionFunc){
                Position T_base = coord->parentPositionFunc() * coord->T;
                T_world = T_base * T;
            } else {
                T_world = coord->T * T;
            }
        } else {
            T_world.linear() = coord->R0 * T.linear();
            T_world.translation() = coord->T * T.translation();
            if(!T_world.linear().isApprox(coord->T.linear())){
                coord->T = T_world;
                Position T_user;
                T_user.linear() = T.linear();
                T_user.translation().setZero();
                positionWidget->setPosition(T_user);
            }
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
    return impl->storeState(archive);
}


bool LocationView::Impl::storeState(Archive& archive)
{
    positionWidget->storeState(archive);
    if(lastCoordIndex < 0){
        if(lastCoordIndex == WorldCoordIndex){
            archive.write("coordinate", "world");
        } else if(lastCoordIndex == ParentCoordIndex){
            archive.write("coordinate", "parent");
        } else if(lastCoordIndex == LocalCoordIndex){
            archive.write("coordinate", "local");
        }
    } else {
        archive.write("coordinate", lastCoordIndex);
    }
    return true;
}


bool LocationView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LocationView::Impl::restoreState(const Archive& archive)
{
    positionWidget->restoreState(archive);

    auto coordNode = archive.find("coordinate");
    if(coordNode->isValid()){
        auto coord = coordNode->toString();
        if(coord == "world"){
            lastCoordIndex = WorldCoordIndex;
        } else if(coord == "parent"){
            lastCoordIndex = ParentCoordIndex;
        } else if(coord == "local"){
            lastCoordIndex = LocalCoordIndex;
        } else {
            lastCoordIndex = coordNode->toInt();
        }
    }
    return true;
}
