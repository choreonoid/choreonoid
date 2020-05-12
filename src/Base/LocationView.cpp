#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "TargetItemPicker.h"
#include "LocatableItem.h"
#include "PositionEditManager.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
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
    enum TargetType { LinkTarget, PositionEditTarget } targetType;
    LocatableItem* targetItem;
    TargetItemPicker<Item> targetItemPicker;
    ScopedConnectionSet targetConnections;
    AbstractPositionEditTarget* positionEditTarget;
    QLabel caption;
    CheckBox lockCheck;
    PositionWidget* positionWidget;
    vector<CoordinateInfoPtr> coordinates;
    QLabel coordinateLabel;
    ComboBox coordinateCombo;
    int lastCoordIndex;
    ScopedConnection activeStateConnection;
        
    Impl(LocationView* self);
    void setTargetItem(Item* item);
    bool setPositionEditTarget(AbstractPositionEditTarget* target);
    void setLocked(bool on);
    void onLockCheckToggled(bool on);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void updatePositionWidgetWithLocatableItemLocation();
    bool setInputPositionToTargetItem(const Position& T_input);
    void updatePositionWidgetWithPositionEditTargetPosition();
    void onPositionEditTargetExpired();
    bool setInputPositionToPositionEditTarget(const Position& T_input);
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
    self->setLayout(vbox, 1.0, 0.5, 1.0, 0.5);

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
        [&](int){ updatePositionWidgetWithLocatableItemLocation(); });
    hbox->addWidget(&coordinateCombo, 1);
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);

    positionWidget->setPositionCallback(
        [&](const Position& T){
            if(targetType == LinkTarget){
                return setInputPositionToTargetItem(T);
            } else if(targetType == PositionEditTarget){
                return setInputPositionToPositionEditTarget(T);
            }
            return false;
        });
    
    vbox->addWidget(positionWidget);

    vbox->addStretch();

    targetItemPicker.setTargetPredicate(
        [](Item* item){
            if(auto locatableItem = dynamic_cast<LocatableItem*>(item)){
                if(locatableItem->getLocationType() != LocatableItem::InvalidLocation){
                    return true;
                }
            }
            return false;
        });

    targetItemPicker.sigTargetItemChanged().connect(
        [&](Item* item){ setTargetItem(item); });

    targetType = LinkTarget;
    targetItem = nullptr;
    setTargetItem(nullptr);
    positionEditTarget = nullptr;
    lastCoordIndex = ParentCoordIndex;
}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onActivated()
{
    auto pem = PositionEditManager::instance();
    
    impl->activeStateConnection =
        pem->sigPositionEditRequest().connect(
            [&](AbstractPositionEditTarget* target){
                return impl->setPositionEditTarget(target);
            });


    if(auto positionEditTarget = pem->lastPositionEditTarget()){
        impl->setPositionEditTarget(positionEditTarget);
    }
}


void LocationView::onDeactivated()
{
    impl->activeStateConnection.disconnect();
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);
}


void LocationView::Impl::setTargetItem(Item* item)
{
    targetConnections.disconnect();
    targetType = LinkTarget;
    targetItem = dynamic_cast<LocatableItem*>(item);
    
    if(!targetItem){
        caption.setText("-----");
        setLocked(false);
        //positionWidget->setPosition(Position::Identity());
        positionWidget->clearPosition();
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();
        
    } else {
        caption.setText(targetItem->getLocationName().c_str());
        setLocked(!targetItem->isLocationEditable());

        targetConnections.add(
            targetItem->sigLocationChanged().connect(
                [this](){ updatePositionWidgetWithLocatableItemLocation(); }));
        
        targetConnections.add(
            targetItem->sigLocationEditableChanged().connect(
                [this](bool on){ setLocked(!on); }));

        targetConnections.add(
            item->sigNameChanged().connect(
                [this](const std::string& /* oldName */){
                    caption.setText(targetItem->getLocationName().c_str());
                }));
        
        updateBaseCoordinateSystems();
        updatePositionWidgetWithLocatableItemLocation();
    }

    lockCheck.blockSignals(false);
}


bool LocationView::Impl::setPositionEditTarget(AbstractPositionEditTarget* target)
{
    targetConnections.disconnect();
    positionWidget->clearPosition();

    caption.setText(target->getPositionName().c_str());
    
    targetType = PositionEditTarget;
    positionEditTarget = target;

    targetConnections.add(
        target->sigPositionChanged().connect(
            [&](const Position&){ updatePositionWidgetWithPositionEditTargetPosition(); }));

    targetConnections.add(
        target->sigPositionEditTargetExpired().connect(
            [&](){ onPositionEditTargetExpired(); }));

    positionWidget->setEnabled(target->isEditable());

    clearBaseCoordinateSystems();
    
    updatePositionWidgetWithPositionEditTargetPosition();

    return true;
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
        targetConnections.block();
        targetItem->setLocationEditable(!on);
        positionWidget->setEditable(!on);
        targetConnections.unblock();
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

    int locationType = targetItem->getLocationType();
    if(locationType != LocatableItem::OffsetLocation){
        auto worldCoord = new CoordinateInfo(_("World"), WorldCoordIndex);
        worldCoord->T.setIdentity();
        coordinates.push_back(worldCoord);
    }

    CoordinateInfo* parentCoord = nullptr;
    
    if(auto parentLocatable = targetItem->getParentLocatableItem()){
        ItemPtr parentItem = parentLocatable->getCorrespondingItem();
        string label;
        if(locationType == LocatableItem::OffsetLocation){
            label = format(_("Offset ( {} )"), parentLocatable->getLocationName());
        } else {
            label = format(_("Parent ( {} )"), parentLocatable->getLocationName());
        }
        parentCoord = new CoordinateInfo(label, ParentCoordIndex);
        // parentItem is captured to keep parentLocatable alive until the function is disposed
        parentCoord->parentPositionFunc =
            [parentItem, parentLocatable](){ return parentLocatable->getLocation(); };

    } else if(locationType == LocatableItem::OffsetLocation){
        parentCoord = new CoordinateInfo(_("Offset"), ParentCoordIndex);
    }
    if(parentCoord){
        defaultComboIndex = coordinates.size();
        coordinates.push_back(parentCoord);
    }

    Position T = targetItem->getLocation();
    auto localCoord = new CoordinateInfo(_("Local"), LocalCoordIndex, T);
    localCoord->isLocal = true;
    localCoord->R0 = T.linear();
    coordinates.push_back(localCoord);

    if(locationType != LocatableItem::OffsetLocation){
        //! \todo Use WorldItem as the root of the item search tree
        auto frameListItems =
            RootItem::instance()->descendantItems<CoordinateFrameListItem>();
        string basename;
        for(auto& frameListItem : frameListItems){
            if(!frameListItem->isForBaseFrames()){
                continue;
            }
            basename.clear();
            function<Position()> parentPositionFunc;
            auto parentItem = frameListItem->getParentLocatableItem();
            if(!parentItem ||
               parentItem->getCorrespondingItem() == targetItem->getCorrespondingItem()){
                continue;
            }
            basename = parentItem->getLocationName();
            basename += " ";
            parentPositionFunc =
                [parentItem](){ return parentItem->getLocation(); };
            basename += frameListItem->displayName();
            basename += " ";
            auto frames = frameListItem->frameList();
            int n = frames->numFrames();
            for(int i=0; i < n; ++i){
                auto frameItem = frameListItem->findFrameItemAt(i);
                if(frameItem && frameItem == targetItem){
                    continue;
                }
                string name(basename);
                auto frame = frames->frameAt(i);
                name += frame->id().label();
                if(!frame->note().empty()){
                    name += " : ";
                    name += frame->note();
                }
                auto coord = new CoordinateInfo(name, coordinates.size(), frame->T(), parentPositionFunc);
                coordinates.push_back(coord);
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


void LocationView::Impl::updatePositionWidgetWithLocatableItemLocation()
{
    if(!targetItem){
        return;
    }

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    lastCoordIndex = coord->index;
    Position T_location = targetItem->getLocation();

    Position T;
    if(!coord->isLocal){
        if(coord->parentPositionFunc){
            Position T_base = coord->parentPositionFunc() * coord->T;
            T = T_base.inverse(Eigen::Isometry) * T_location;
        } else {
            T = coord->T.inverse(Eigen::Isometry) * T_location;
        }
    } else {
        Matrix3 R_prev = coord->T.linear();
        Matrix3 R_current = T_location.linear();
        if(!R_current.isApprox(R_prev)){
            coord->T = T_location;
        }
        T = coord->T.inverse(Eigen::Isometry) * T_location;
        T.linear() = coord->R0.transpose() * T_location.linear();
    }
    positionWidget->setPosition(T);
}


bool LocationView::Impl::setInputPositionToTargetItem(const Position& T_input)
{
    if(targetItem){
        auto& coord = coordinates[coordinateCombo.currentIndex()];
        Position T_location;
        if(!coord->isLocal){
            if(coord->parentPositionFunc){
                Position T_base = coord->parentPositionFunc() * coord->T;
                T_location = T_base * T_input;
            } else {
                T_location = coord->T * T_input;
            }
        } else {
            T_location.linear() = coord->R0 * T_input.linear();
            T_location.translation() = coord->T * T_input.translation();
            if(!T_location.linear().isApprox(coord->T.linear())){
                coord->T = T_location;
                Position T_user;
                T_user.linear() = T_input.linear();
                T_user.translation().setZero();
                positionWidget->setPosition(T_user);
            }
        }
        targetConnections.block();
        targetItem->setLocation(T_location);
        targetConnections.unblock();
        return true;
    }
    return false;
}


void LocationView::Impl::updatePositionWidgetWithPositionEditTargetPosition()
{
    positionWidget->setReferenceRpy(Vector3::Zero());
    positionWidget->setPosition(positionEditTarget->getPosition());
}


void LocationView::Impl::onPositionEditTargetExpired()
{

}
        

bool LocationView::Impl::setInputPositionToPositionEditTarget(const Position& T_input)
{
    bool accepted = false;
    if(positionEditTarget){
        targetConnections.block();
        accepted = positionEditTarget->setPosition(T_input);
        targetConnections.unblock();
    }
    return accepted;
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
