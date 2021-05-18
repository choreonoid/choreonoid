#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "ItemList.h"
#include "LocatableItem.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "RootItem.h"
#include "UnifiedEditHistory.h"
#include "CheckBox.h"
#include "ComboBox.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <QBoxLayout>
#include <QLabel>
#include <fmt/format.h>
#include <unordered_set>
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
    Isometry3 T;
    std::function<Isometry3()> parentPositionFunc;
    Matrix3 R0; // the initial rotation of the local coordinate system
    bool isLocal;
    CoordinateInfo(const std::string& name, int type, int index, const Isometry3& T,
                   const std::function<Isometry3()>& parentPositionFunc)                   
        : name(name), type(type), index(index), T(T), parentPositionFunc(parentPositionFunc), isLocal(false) { }
    CoordinateInfo(const std::string& name, int type, int index, const Isometry3& T)
        : name(name), type(type), index(index), T(T), isLocal(false) { }
    CoordinateInfo(const std::string& name, int type, int index)
        : CoordinateInfo(name, type, index, Isometry3::Identity()) { }
};

typedef ref_ptr<CoordinateInfo> CoordinateInfoPtr;

class LockCheckBox : public CheckBox
{
public:
    LocationView::Impl* impl;
    LockCheckBox(LocationView::Impl* impl) : impl(impl) { }
    bool isChangable();
    virtual void nextCheckState() override;
};

}

namespace cnoid {

class LocationView::Impl
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LocationView* self;

    struct LocationInfo : public Referenced
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LocationProxyPtr proxy;
        LocationProxyPtr parentProxy;
        int type;
        bool isRelativeLocation;
        bool isLocationAbleToBeGlobal;
        bool isDependingOnAnotherTargetLocation;
        ItemPtr item;
        ScopedConnection locationChangeConnection;
        ScopedConnectionSet miscConnections;
        Isometry3 T_primary_relative;
    };
    typedef ref_ptr<LocationInfo> LocationInfoPtr;
    vector<LocationInfoPtr> locations;
    unordered_set<LocationProxy*> locationProxySet;
    Isometry3 T_primary; // The latest primary object global location

    Connection itemSelectionConnection;
    Connection editRequestConnection;
    
    QLabel captionLabel;
    LockCheckBox lockCheck;
    PositionWidget* positionWidget;
    vector<CoordinateInfoPtr> coordinates;
    QLabel coordinateLabel;
    ComboBox coordinateCombo;
    int lastCoordIndex;
    CheckBox individualRotationCheck;
        
    Impl(LocationView* self);
    void onSelectedItemsChanged(const ItemList<>& items);
    bool onEditRequest(LocationProxyPtr location);
    void addLocation(LocationProxyPtr locationProxy, Item* Item);
    void onObjectLocationChanged(LocationInfo* location);
    void updatePrimaryRelativePosition(LocationInfo* location);
    void onLocationExpired(LocationInfoPtr location);
    void setupInterfaceForNewLocations();
    bool checkDependencyOnAnotherLocation(LocationProxyPtr parentProxy);
    void setEditorLocked(bool on);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void updatePositionWidgetWithPrimaryLocation();
    bool updateTargetLocationWithInputPosition(const Isometry3& T_input);
    void finishLocationEditing();
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
    hbox->addWidget(&captionLabel, 1);
    lockCheck.setText(_("Lock"));
    hbox->addWidget(&lockCheck);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    coordinateLabel.setText(_("Coord :"));
    hbox->addWidget(&coordinateLabel, 0);
    coordinateCombo.sigAboutToShowPopup().connect(
        [&](){ updateBaseCoordinateSystems(); });
    coordinateCombo.sigActivated().connect(
        [&](int){ updatePositionWidgetWithPrimaryLocation(); });
    hbox->addWidget(&coordinateCombo, 1);
    vbox->addLayout(hbox);

    positionWidget = new PositionWidget(self);
    positionWidget->setCallbacks(
        [&](const Isometry3& T){ return updateTargetLocationWithInputPosition(T); },
        [&](){ finishLocationEditing(); });
    vbox->addWidget(positionWidget);

    hbox = new QHBoxLayout;
    hbox->addStretch();
    individualRotationCheck.setText(_("Individual Rotation"));
    individualRotationCheck.setEnabled(false);
    hbox->addWidget(&individualRotationCheck);
    vbox->addLayout(hbox);

    vbox->addStretch();

    setupInterfaceForNewLocations();
    
    lastCoordIndex = ParentCoordIndex;
}


LocationView::~LocationView()
{
    delete impl;
}


void LocationView::onActivated()
{
    auto rootItem = RootItem::instance();

    impl->itemSelectionConnection =
        rootItem->sigSelectedItemsChanged().connect(
            [this](const ItemList<>& items){ impl->onSelectedItemsChanged(items); });

    impl->editRequestConnection =
        LocationProxy::sigEditRequest().connect(
            [this](LocationProxy* locationProxy){ return impl->onEditRequest(locationProxy); });

    impl->onSelectedItemsChanged(rootItem->selectedItems());
}


void LocationView::onDeactivated()
{
    impl->itemSelectionConnection.disconnect();
    impl->editRequestConnection.disconnect();
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);
}


void LocationView::Impl::onSelectedItemsChanged(const ItemList<>& items)
{
    for(auto& item : items){
        if(auto locatable = dynamic_cast<LocatableItem*>(item.get())){
            if(auto proxy = locatable->getLocationProxy()){
                if(locationProxySet.find(proxy) == locationProxySet.end()){
                    addLocation(proxy, item);
                }
            }
        }
    }
    
    // Arrange location objects
    if(!locations.empty()){
        auto oldFirstLocation = locations.front();
        auto it = locations.begin();
        while(it != locations.end()){
            auto& location = *it;
            if(!location->item || !location->item->isSelected()){
                locationProxySet.erase(location->proxy);
                it = locations.erase(it);
            } else {
                ++it;
            }
        }
        // Keep the last primary location object if there is no selection
        if(locations.empty()){
            locations.push_back(oldFirstLocation);
            if(oldFirstLocation->item){
                locationProxySet.insert(oldFirstLocation->proxy);
            }
        }
    }

    setupInterfaceForNewLocations();
}


bool LocationView::Impl::onEditRequest(LocationProxyPtr locationProxy)
{
    locations.clear();
    locationProxySet.clear();
    addLocation(locationProxy, nullptr);
    setupInterfaceForNewLocations();
    return true;
}


void LocationView::Impl::addLocation(LocationProxyPtr locationProxy, Item* ownerItem)
{
    int type = locationProxy->locationType();
    if(type == LocationProxy::InvalidLocation){
        return;
    }

    // The variable must not be a smart pointer to avoid a circular reference
    // caused by a captured variable in the following lambda expressions
    auto location = new LocationInfo;
    location->proxy = locationProxy;
    location->parentProxy = locationProxy->getParentLocationProxy();
    location->type = type;

    location->isRelativeLocation = false;
    if(location->type == LocationProxy::ParentRelativeLocation && location->parentProxy){
        location->isRelativeLocation = true;
    } else if(location->type == LocationProxy::OffsetLocation){
        location->isRelativeLocation = true;
    }
    location->isLocationAbleToBeGlobal = (!location->isRelativeLocation || location->parentProxy);

    location->isDependingOnAnotherTargetLocation = false;
        
    location->locationChangeConnection.reset(
        locationProxy->sigLocationChanged().connect(
            [this, location](){ onObjectLocationChanged(location); }));
    
    location->miscConnections.add(
        locationProxy->sigAttributeChanged().connect(
            [this, location](){
                if(location == locations.front() || location == locations[1]){
                    setupInterfaceForNewLocations();
                }
            }));

    location->miscConnections.add(
        locationProxy->sigExpired().connect(
            [this, location](){ onLocationExpired(location); }));

    if(ownerItem){
        location->miscConnections.add(
            ownerItem->sigDisconnectedFromRoot().connect(
                [this, location](){ onLocationExpired(location); }));
        location->item = ownerItem;
    }
        
    locations.push_back(location);
    locationProxySet.insert(locationProxy);
}


void LocationView::Impl::onObjectLocationChanged(LocationInfo* location)
{
    if(location == locations.front()){
        updatePositionWidgetWithPrimaryLocation();
    } else {
        updatePrimaryRelativePosition(location);
    }
}


void LocationView::Impl::updatePrimaryRelativePosition(LocationInfo* location)
{
    if(location->type != LocationProxy::OffsetLocation){
        location->T_primary_relative =
            T_primary.inverse(Eigen::Isometry) * location->proxy->getGlobalLocation();
    }
}


void LocationView::Impl::onLocationExpired(LocationInfoPtr location)
{
    auto it = std::find(locations.begin(), locations.end(), location);
    if(it != locations.end()){
        locations.erase(it);
    }
    locationProxySet.erase(location->proxy);
    setupInterfaceForNewLocations();
}
        

void LocationView::Impl::setupInterfaceForNewLocations()
{
    if(locations.empty()){
        captionLabel.setText("-----");
        setEditorLocked(false);
        positionWidget->clearPosition();
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();

    } else {
        auto& location = locations.front();
        auto& proxy = location->proxy;

        captionLabel.setText(proxy->getName().c_str());
        int n = locations.size();
        if(n == 1){
            captionLabel.setText(
                format(_("<b>{0}</b>"), proxy->getName()).c_str());
        } else if(n == 2){
            captionLabel.setText(
                format(_("<b>{0}</b> + {1}"),
                       proxy->getName(), locations[1]->proxy->getName()).c_str());
        } else {
            captionLabel.setText(
                format(_("<b>{0}</b> + {1} objects"),
                       proxy->getName(), n - 1).c_str());
        }
        individualRotationCheck.setEnabled(n >= 2);

        // Update the dependency information
        for(auto& location : locations){
            location->isDependingOnAnotherTargetLocation =
                checkDependencyOnAnotherLocation(location->parentProxy);
        }

        updateBaseCoordinateSystems();
        updatePositionWidgetWithPrimaryLocation();

        bool locked = !location->proxy->isEditable();

        if(locations.size() >= 2){
            if(location->type == LocationProxy::OffsetLocation){
                locked = true;
            } else {
                for(size_t i=1; i < locations.size(); ++i){
                    auto& subLocation = locations[i];
                    if(subLocation->type == LocationProxy::OffsetLocation ||
                       !subLocation->proxy->isEditable()){
                        locked = true;
                        break;
                    }
                    updatePrimaryRelativePosition(subLocation);
                }
            }
        }
        
        setEditorLocked(locked);
    }
}


bool LocationView::Impl::checkDependencyOnAnotherLocation(LocationProxyPtr parentProxy)
{
    while(parentProxy){
        if(locationProxySet.find(parentProxy) != locationProxySet.end()){
            return true;
        }
        parentProxy = parentProxy->getParentLocationProxy();
    }
    return false;
}


void LocationView::Impl::setEditorLocked(bool on)
{
    lockCheck.blockSignals(true);
    positionWidget->setEditable(!on);
    lockCheck.setChecked(on);
    lockCheck.setEnabled(lockCheck.isChangable());
    lockCheck.blockSignals(false);
}


namespace {

bool LockCheckBox::isChangable()
{
    auto& locations = impl->locations;
    if(locations.empty()){
        return false;
    }
    int n = locations.size();
    bool changable = true;
    if(n >= 2){
        if(locations.front()->type == LocationProxy::OffsetLocation){
            changable = false;
        } else {
            for(int i=1; i < n; ++i){
                auto& location = locations[i];
                if(location->type == LocationProxy::OffsetLocation){
                    changable = false;
                    break;
                }
            }
        }
    }
    return changable;
}
    

void LockCheckBox::nextCheckState()
{
    if(isChangable()){
        bool isLocked = isChecked();
        for(auto& location : impl->locations){
            location->miscConnections.block();
            location->proxy->setEditable(isLocked);
            location->miscConnections.unblock();
        }
        impl->setupInterfaceForNewLocations();
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
    if(locations.empty()){
        return;
    }
    LocationInfoPtr location = locations.front();
    
    clearBaseCoordinateSystems();
    int defaultComboIndex = 0;

    if(location->isLocationAbleToBeGlobal && (location->type != LocationProxy::OffsetLocation)){
        auto worldCoord = new CoordinateInfo(_("World"), BaseCoord, WorldCoordIndex);
        worldCoord->T.setIdentity();
        coordinates.push_back(worldCoord);
    }

    if(!location->isDependingOnAnotherTargetLocation){
        CoordinateInfo* parentCoord = nullptr;
        if(location->parentProxy){
            string label;
            if(location->type == LocationProxy::OffsetLocation){
                label = format(_("Offset ( {} )"), location->parentProxy->getName());
            } else {
                label = format(_("Parent ( {} )"), location->parentProxy->getName());
            }
            parentCoord = new CoordinateInfo(label, ParentCoord, ParentCoordIndex);
            parentCoord->parentPositionFunc =
                [location](){ return location->parentProxy->getGlobalLocation(); };
            
        } else if(location->isRelativeLocation){
            if(location->type == LocationProxy::OffsetLocation){
                parentCoord = new CoordinateInfo(_("Offset"), ParentCoord, ParentCoordIndex);
            } else {
                parentCoord = new CoordinateInfo(_("Parent"), ParentCoord, ParentCoordIndex);
            }
        }
        if(parentCoord){
            defaultComboIndex = coordinates.size();
            coordinates.push_back(parentCoord);
        }
    }

    Isometry3 T = location->proxy->getLocation();
    auto localCoord = new CoordinateInfo(_("Local"), LocalCoord, LocalCoordIndex, T);
    localCoord->isLocal = true;
    localCoord->R0 = T.linear();
    coordinates.push_back(localCoord);

    if(location->isLocationAbleToBeGlobal && (location->type != LocationProxy::OffsetLocation)){

        //! \todo Use WorldItem as the root of the item search tree
        auto frameListItems =  RootItem::instance()->descendantItems<CoordinateFrameListItem>();
        string basename;
        
        for(auto& frameListItem : frameListItems){
            if(!frameListItem->isForBaseFrames()){
                continue;
            }
            basename.clear();
            function<Isometry3()> parentPositionFunc;
            auto frameParentLocation = frameListItem->getFrameParentLocationProxy();

            if(checkDependencyOnAnotherLocation(frameParentLocation)){
                continue;
            }
            
            basename = frameParentLocation->getName();
            basename += " ";
            parentPositionFunc = [frameParentLocation](){ return frameParentLocation->getGlobalLocation(); };
            basename += frameListItem->displayName();
            basename += " ";
            auto frames = frameListItem->frameList();
            int n = frames->numFrames();

            Item* targetItem = location->item;
            if(!targetItem){
                targetItem = location->proxy->getCorrespondingItem();
            }
            
            for(int i=0; i < n; ++i){
                if(auto frameItem = frameListItem->findFrameItemAt(i)){
                    if(frameItem == targetItem){
                        continue;
                    }
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


void LocationView::Impl::updatePositionWidgetWithPrimaryLocation()
{
    if(locations.empty()){
        return;
    }
    auto& location = locations.front();

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    lastCoordIndex = coord->index;
    Isometry3 T_display;

    if(coord->type == LocalCoord){
        Isometry3 T_location = location->proxy->getLocation();
        Matrix3 R_prev = coord->T.linear();
        Matrix3 R_current = T_location.linear();
        // When the rotation is changed, the translation origin is reset
        if(!R_current.isApprox(R_prev, 1e-6)){
            coord->T = T_location;
        }
        T_display.translation() = (coord->T.inverse(Eigen::Isometry) * T_location).translation();
        T_display.linear() = coord->R0.transpose() * T_location.linear();

    } else {
        Isometry3 T_location_global;
        bool isGlobalBase = false;

        if(location->isRelativeLocation){
            if(coord->type == ParentCoord){
                T_display = location->proxy->getLocation();

            } else if(location->parentProxy){
                Isometry3 T_parent = location->parentProxy->getGlobalLocation();
                T_location_global = T_parent * location->proxy->getLocation();
                isGlobalBase = true;
            } else {
                return; // invalid case
            }
        } else {
            T_location_global = location->proxy->getLocation();
            isGlobalBase = true;
        }
        if(isGlobalBase){
            if(coord->parentPositionFunc){
                Isometry3 T_base = coord->parentPositionFunc() * coord->T;
                T_display = T_base.inverse(Eigen::Isometry) * T_location_global;
            } else {
                T_display = coord->T.inverse(Eigen::Isometry) * T_location_global;
            }
        }
    }
    
    // positionWidget->setReferenceRpy(Vector3::Zero());
    positionWidget->setPosition(T_display);

    T_primary = location->proxy->getGlobalLocation();
}


bool LocationView::Impl::updateTargetLocationWithInputPosition(const Isometry3& T_input)
{
    if(locations.empty()){
        return false;
    }
    auto& location = locations.front();

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    Isometry3 T_location;

    if(coord->type == LocalCoord){
        T_location.linear() = coord->R0 * T_input.linear();
        T_location.translation() = coord->T * T_input.translation();
        if(!T_location.linear().isApprox(coord->T.linear(), 1e-6)){
            coord->T = T_location;
            Isometry3 T_display;
            T_display.linear() = T_input.linear();
            T_display.translation().setZero();
            positionWidget->setPosition(T_display);
        }
    } else {
        if(location->isRelativeLocation && (coord->type == ParentCoord)){
            T_location = T_input;
        } else {
            Isometry3 T_base;
            if(coord->parentPositionFunc){
                T_base = coord->parentPositionFunc() * coord->T;
            } else {
                T_base.setIdentity();
            }
            if(location->isRelativeLocation){
                if(location->parentProxy){
                    Isometry3 T_global = T_base * T_input;
                    Isometry3 T_parent = location->parentProxy->getGlobalLocation();
                    T_location = T_parent.inverse(Eigen::Isometry) * T_global;
                } else {
                    return false; // invalid case
                }
            } else {
                T_location = T_base * T_input;
            }
        }
    }

    normalizeRotation(T_location);

    for(auto& loc : locations){
        loc->locationChangeConnection.block();
    }

    bool updated = false;

    // Synchronize the positions of the remaining location objects
    if(locations.size() == 1){
        updated = location->proxy->setLocation(T_location);

    } else if(locations.size() >= 2){
        Isometry3 T_global = location->proxy->getGlobalLocationOf(T_location);
        for(size_t i=1; i < locations.size(); ++i){
            auto& subLocation = locations[i];
            if(!subLocation->isDependingOnAnotherTargetLocation){
                Isometry3 T;
                if(!individualRotationCheck.isChecked()){
                    T = T_global * subLocation->T_primary_relative;
                } else {
                    T.linear() = T_global.linear() * subLocation->T_primary_relative.linear();
                    T.translation() = T_global.translation() + subLocation->T_primary_relative.translation();
                }
                normalizeRotation(T);
                subLocation->proxy->setGlobalLocation(T);
            }
        }
        updated = location->proxy->setGlobalLocation(T_global);
    }

    for(auto& loc : locations){
        loc->locationChangeConnection.unblock();
    }
    
    return updated;
}


void LocationView::Impl::finishLocationEditing()
{
    if(locations.size() == 1){
        locations.front()->proxy->finishLocationEditing();

    } else if(locations.size() >= 2){
        auto history = UnifiedEditHistory::instance();
        history->beginEditGroup(_("Change multiple object locations"));
        for(auto& location : locations){
            location->proxy->finishLocationEditing();
        }
        history->endEditGroup();
    }
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
