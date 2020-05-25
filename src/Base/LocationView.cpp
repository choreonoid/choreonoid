#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "TargetItemPicker.h"
#include "LocatableItem.h"
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

enum CoordinateType { BaseCoord, ParentCoord, LocalCoord };

enum CoordinateIndexId {
    WorldCoordIndex = -2,
    ParentCoordIndex = -3,
    LocalCoordIndex = -4
};

struct CoordinateInfo : public Referenced
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    string name;
    int type;
    int index;
    Position T;
    std::function<Position()> parentPositionFunc;
    Matrix3 R0; // the initial rotation of the local coordinate system
    bool isLocal;
    CoordinateInfo(
        const std::string& name, int type, int index, const Position& T, const std::function<Position()>& func)
        : name(name), type(type), index(index), T(T), parentPositionFunc(func), isLocal(false) { }
    CoordinateInfo(const std::string& name, int type, int index, const Position& T)
        : CoordinateInfo(name, type, index, T, nullptr) { }
    CoordinateInfo(const std::string& name, int type, int index)
        : CoordinateInfo(name, type, index, Position::Identity(), nullptr) { }
};

typedef ref_ptr<CoordinateInfo> CoordinateInfoPtr;

class LockCheckBox : public CheckBox
{
public:
    LocationView::Impl* impl;
    LockCheckBox(LocationView::Impl* impl) : impl(impl) { }
    virtual void nextCheckState() override;
};

}

namespace cnoid {

class LocationView::Impl
{
public:
    LocationView* self;
    LocatableItem* locatableItem;
    LocatableItem* parentLocatableItem;
    int locationType;
    bool isRelativeLocation;
    bool isLocationAbleToBeGlobal;
    TargetItemPicker<Item> targetItemPicker;
    ScopedConnectionSet locatableItemConnections;
    QLabel caption;
    LockCheckBox lockCheck;
    PositionWidget* positionWidget;
    vector<CoordinateInfoPtr> coordinates;
    QLabel coordinateLabel;
    ComboBox coordinateCombo;
    int lastCoordIndex;
    ScopedConnection activeStateConnection;
        
    Impl(LocationView* self);
    bool setLocatableItem(LocatableItem* item);
    void setEditorLocked(bool on);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void updatePositionWidgetWithLocatableItemLocation();
    bool setInputPositionToTargetItem(const Position& T_input);
    void onLocationExpired();
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
    : self(self),
      lockCheck(this)
{
    self->setDefaultLayoutArea(View::CENTER);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox, 1.0, 0.5, 1.0, 0.5);

    auto hbox = new QHBoxLayout;
    caption.setStyleSheet("font-weight: bold");
    hbox->addWidget(&caption, 1);
    lockCheck.setText(_("Lock"));
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
        [&](const Position& T){ return setInputPositionToTargetItem(T); });
    
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
        [&](Item* item){ setLocatableItem(dynamic_cast<LocatableItem*>(item)); });

    locatableItem = nullptr;
    setLocatableItem(nullptr);
    lastCoordIndex = ParentCoordIndex;
}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onActivated()
{
    impl->activeStateConnection =
        LocatableItem::sigLocationEditRequest().connect(
            [&](LocatableItem* item){ return impl->setLocatableItem(item); });
}


void LocationView::onDeactivated()
{
    impl->activeStateConnection.disconnect();
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);
}


bool LocationView::Impl::setLocatableItem(LocatableItem* item)
{
    locatableItemConnections.disconnect();
    locatableItem = item;
    
    if(!locatableItem){
        parentLocatableItem = nullptr;
        locationType = LocatableItem::InvalidLocation;
        caption.setText("-----");
        setEditorLocked(false);
        positionWidget->clearPosition();
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();
        
    } else {
        parentLocatableItem = locatableItem->getParentLocatableItem();
        locationType = locatableItem->getLocationType();
        isRelativeLocation =
            (locationType == LocatableItem::ParentRelativeLocation ||
             locationType == LocatableItem::OffsetLocation);
        isLocationAbleToBeGlobal = (!isRelativeLocation || parentLocatableItem);
        
        caption.setText(locatableItem->getLocationName().c_str());
        setEditorLocked(!locatableItem->isLocationEditable());

        locatableItemConnections.add(
            locatableItem->sigLocationChanged().connect(
                [this](){ updatePositionWidgetWithLocatableItemLocation(); }));
        
        locatableItemConnections.add(
            locatableItem->sigLocationAttributeChanged().connect(
                [this](){
                    caption.setText(locatableItem->getLocationName().c_str());
                    setEditorLocked(!locatableItem->isLocationEditable());
                }));

        locatableItemConnections.add(
            item->sigLocationExpired().connect(
                [this](){ onLocationExpired(); }));
        
        updateBaseCoordinateSystems();
        updatePositionWidgetWithLocatableItemLocation();
    }

    lockCheck.blockSignals(false);

    return locatableItem != nullptr;
}


void LocationView::Impl::setEditorLocked(bool on)
{
    lockCheck.blockSignals(true);
    positionWidget->setEditable(!on);
    lockCheck.setChecked(on);
    lockCheck.blockSignals(false);
}


void LockCheckBox::nextCheckState()
{
    auto& locatable = impl->locatableItem;
    if(locatable){
        bool doLock = !isChecked();
        locatable->setLocationEditable(!doLock);
        if(locatable->isLocationEditable() == !doLock){
            impl->setEditorLocked(doLock);
        }
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
    if(!locatableItem){
        return;
    }
    
    clearBaseCoordinateSystems();
    int defaultComboIndex = 0;

    if(isLocationAbleToBeGlobal && (locationType != LocatableItem::OffsetLocation)){
        auto worldCoord = new CoordinateInfo(_("World"), BaseCoord, WorldCoordIndex);
        worldCoord->T.setIdentity();
        coordinates.push_back(worldCoord);
    }

    CoordinateInfo* parentCoord = nullptr;
    
    if(parentLocatableItem){
        ItemPtr parentItem = parentLocatableItem->getCorrespondingItem();
        string label;
        if(locationType == LocatableItem::OffsetLocation){
            label = format(_("Offset ( {} )"), parentLocatableItem->getLocationName());
        } else {
            label = format(_("Parent ( {} )"), parentLocatableItem->getLocationName());
        }
        parentCoord = new CoordinateInfo(label, ParentCoord, ParentCoordIndex);
        // parentItem is captured to keep parentLocatable alive until the function is disposed
        parentCoord->parentPositionFunc =
            [this, parentItem](){ return parentLocatableItem->getLocation(); };

    } else if(isRelativeLocation){
        if(locationType == LocatableItem::OffsetLocation){
            parentCoord = new CoordinateInfo(_("Offset"), ParentCoord, ParentCoordIndex);
        } else {
            parentCoord = new CoordinateInfo(_("Parent"), ParentCoord, ParentCoordIndex);
        }
    }
    if(parentCoord){
        defaultComboIndex = coordinates.size();
        coordinates.push_back(parentCoord);
    }

    Position T = locatableItem->getLocation();
    auto localCoord = new CoordinateInfo(_("Local"), LocalCoord, LocalCoordIndex, T);
    localCoord->isLocal = true;
    localCoord->R0 = T.linear();
    coordinates.push_back(localCoord);

    if(isLocationAbleToBeGlobal && (locationType != LocatableItem::OffsetLocation)){
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
               parentItem->getCorrespondingItem() == locatableItem->getCorrespondingItem()){
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
                if(frameItem && frameItem == locatableItem){
                    continue;
                }
                string name(basename);
                auto frame = frames->frameAt(i);
                name += frame->id().label();
                if(!frame->note().empty()){
                    name += " : ";
                    name += frame->note();
                }
                auto coord = new CoordinateInfo(name, BaseCoord, coordinates.size(), frame->T(), parentPositionFunc);
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
    if(!locatableItem){
        return;
    }

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    lastCoordIndex = coord->index;
    Position T_display;

    if(coord->type == LocalCoord){
        Position T_location = locatableItem->getLocation();
        Matrix3 R_prev = coord->T.linear();
        Matrix3 R_current = T_location.linear();
        if(!R_current.isApprox(R_prev)){
            coord->T = T_location;
        }
        T_display = coord->T.inverse(Eigen::Isometry) * T_location;
        T_display.linear() = coord->R0.transpose() * T_location.linear();

    } else {
        Position T_location_global;
        bool isGlobalBase = false;

        if(isRelativeLocation){
            if(coord->type == ParentCoord){
                T_display = locatableItem->getLocation();

            } else if(parentLocatableItem){
                Position T_relative = parentLocatableItem->getLocation();
                T_location_global = T_relative * locatableItem->getLocation();
                isGlobalBase = true;
            } else {
                return; // invalid case
            }
        } else {
            T_location_global = locatableItem->getLocation();
            isGlobalBase = true;
        }
        if(isGlobalBase){
            if(coord->parentPositionFunc){
                Position T_base = coord->parentPositionFunc() * coord->T;
                T_display = T_base.inverse(Eigen::Isometry) * T_location_global;
            } else {
                T_display = coord->T.inverse(Eigen::Isometry) * T_location_global;
            }
        }
    }
    
    // positionWidget->setReferenceRpy(Vector3::Zero());
    positionWidget->setPosition(T_display);
}


bool LocationView::Impl::setInputPositionToTargetItem(const Position& T_input)
{
    if(!locatableItem){
        return false;
    }

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    Position T_location;

    if(coord->type == LocalCoord){
        T_location.linear() = coord->R0 * T_input.linear();
        T_location.translation() = coord->T * T_input.translation();
        if(!T_location.linear().isApprox(coord->T.linear())){
            coord->T = T_location;
            Position T_display;
            T_display.linear() = T_input.linear();
            T_display.translation().setZero();
            positionWidget->setPosition(T_display);
        }
    } else {
        if(isRelativeLocation && (coord->type == ParentCoord)){
            T_location = T_input;
        } else {
            Position T_base;
            if(coord->parentPositionFunc){
                T_base = coord->parentPositionFunc() * coord->T;
            } else {
                T_base.setIdentity();
            }
            if(isRelativeLocation){
                if(parentLocatableItem){
                    Position T_global = T_base * T_input;
                    Position T_parent = parentLocatableItem->getLocation();
                    T_location = T_parent.inverse(Eigen::Isometry) * T_global;
                } else {
                    return false; // invalid case
                }
            } else {
                T_location = T_base * T_input;
            }
        }
    }    

    locatableItemConnections.block();
    locatableItem->setLocation(T_location);
    locatableItemConnections.unblock();
    
    return true;
}


void LocationView::Impl::onLocationExpired()
{
    setLocatableItem(nullptr);
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
