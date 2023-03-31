#include "LocationView.h"
#include "PositionWidget.h"
#include "ViewManager.h"
#include "ItemList.h"
#include "LocatableItem.h"
#include "CoordinateFrameListItem.h"
#include "CoordinateFrameItem.h"
#include "RootItem.h"
#include "UnifiedEditHistory.h"
#include "Archive.h"
#include "LazyCaller.h"
#include "CheckBox.h"
#include "ComboBox.h"
#include <cnoid/CoordinateFrameList>
#include <cnoid/ConnectionSet>
#include <cnoid/EigenUtil>
#include <QBoxLayout>
#include <QLabel>
#include <fmt/format.h>
#include <set>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

struct LocationInfo : public Referenced
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    LocationProxyPtr location;
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

struct LocationCategory : public Referenced
{
    string name;
    vector<LocationInfoPtr> locationInfos;
    bool changed;
};
typedef ref_ptr<LocationCategory> LocationCategoryPtr;

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

    vector<LocationCategoryPtr> locationCategories;
    LocationCategoryPtr currentCategory;
    set<LocationProxy*> managedLocations;
    LazyCaller setupInterfaceForNewLocationsLater;
    Isometry3 T_primary; // The latest primary object global location
    Connection itemSelectionConnection;
    Connection editRequestConnection;
    map<ItemPtr, ScopedConnection> itemConnectionMap;
    
    QLabel singleCategoryLabel;
    ComboBox categoryCombo;
    string preferredCategoryName;
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
    void clearLocationCategories();
    void addLocation(LocationProxyPtr location, Item* Item);
    void onObjectLocationChanged(LocationInfo* locationInfo);
    void updatePrimaryRelativePosition(LocationInfo* locationInfo);
    void onLocationExpired(LocationInfoPtr locationInfo);
    void setupInterfaceForNewLocations();
    bool checkDependencyOnAnotherLocation(LocationCategory* category, LocationProxyPtr parentLocation);
    void setCurrentLocationCategory(int categoryIndex);
    void setEditorLocked(bool locked, bool blocked);
    void clearBaseCoordinateSystems();
    void updateBaseCoordinateSystems();
    void setIndividualRotationMode(bool on);
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
        N_("LocationView"), N_("Location"), ViewManager::Permanent);
}


LocationView::LocationView()
{
    impl = new Impl(this);
}


LocationView::Impl::Impl(LocationView* self)
    : self(self),
      lockCheck(this)
{
    self->setDefaultLayoutArea(TopRightArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox, 1.0, 0.5, 1.0, 0.5);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(&singleCategoryLabel, 1);
    categoryCombo.hide();
    categoryCombo.sigCurrentIndexChanged().connect(
        [&](int index){ setCurrentLocationCategory(index); });
    hbox->addWidget(&categoryCombo, 1);
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
    individualRotationCheck.sigToggled().connect(
        [&](bool on){ setIndividualRotationMode(on); });
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
            [this](LocationProxy* location){ return impl->onEditRequest(location); });

    impl->onSelectedItemsChanged(rootItem->selectedItems());
}


void LocationView::onDeactivated()
{
    impl->itemSelectionConnection.disconnect();
    impl->editRequestConnection.disconnect();
    impl->itemConnectionMap.clear();
}


void LocationView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->positionWidget->setOptionMenuTo(menuManager);
}


void LocationView::Impl::onSelectedItemsChanged(const ItemList<>& items)
{
    set<LocationProxy*> newLocations;

    for(auto& item : items){
        if(auto locatable = dynamic_cast<LocatableItem*>(item.get())){
            auto locations = locatable->getLocationProxies();
            if(!locations.empty()){
                for(auto& location : locations){
                    if(managedLocations.find(location) == managedLocations.end()){
                        addLocation(location, item);
                    }
                    newLocations.insert(location);
                }
                auto& connection = itemConnectionMap[item];
                if(!connection.connected()){
                    connection.reset(
                        locatable->getSigLocationProxiesChanged().connect(
                            [&](){ onSelectedItemsChanged(RootItem::instance()->selectedItems()); }));
                }
            } else if(auto location = locatable->getLocationProxy()){
                if(managedLocations.find(location) == managedLocations.end()){
                    addLocation(location, item);
                }
                newLocations.insert(location);
            }
        }
    }

    if(newLocations.empty()){
        if(currentCategory){
            // Keep the last primary location object if there is no selection
            newLocations.insert(currentCategory->locationInfos.front()->location);
        }
    }
        
    // Remove inactive locations
    if(!locationCategories.empty()){
        LocationInfoPtr prevPrimayLocationInfo;
        if(currentCategory){
            prevPrimayLocationInfo = currentCategory->locationInfos.front();
        }
        bool firstCategoryBeingChangedFound = false;
        auto p = locationCategories.begin();
        while(p != locationCategories.end()){
            auto& category = *p;
            auto& locationInfos = category->locationInfos;
            auto q = locationInfos.begin();
            while(q != locationInfos.end()){
                auto& location = (*q)->location;
                if(newLocations.find(location) == newLocations.end()){
                    q = locationInfos.erase(q);
                    category->changed = true;
                } else {
                    ++q;
                }
            }
            if(!firstCategoryBeingChangedFound && category->changed){
                // To make the category that last changed in the target locations current for usability reasons.
                preferredCategoryName = category->name;
                firstCategoryBeingChangedFound = true;
            }
            category->changed = false;

            if(locationInfos.empty()){
                p = locationCategories.erase(p);
            } else {
                ++p;
            }
        }
    }

    if(!managedLocations.empty() || !newLocations.empty()){

        // Remove the signal connections of the invalid items
        auto it = itemConnectionMap.begin();
        while(it != itemConnectionMap.end()){
            auto& item = it->first;
            if(!item->isSelected()){
                it = itemConnectionMap.erase(it);
            } else {
                ++it;
            }
        }

        managedLocations.swap(newLocations);

        setupInterfaceForNewLocations();
    }
}


bool LocationView::Impl::onEditRequest(LocationProxyPtr location)
{
    clearLocationCategories();
    addLocation(location, nullptr);
    managedLocations.insert(location);
    setupInterfaceForNewLocations();
    return true;
}


void LocationView::Impl::clearLocationCategories()
{
    locationCategories.clear();
    managedLocations.clear();
    currentCategory.reset();
}


void LocationView::Impl::addLocation(LocationProxyPtr location, Item* ownerItem)
{
    int type = location->locationType();
    if(type == LocationProxy::InvalidLocation){
        return;
    }

    // Note that the variable must not be a smart pointer to avoid a circular reference
    // caused by a captured variable in the following lambda expressions
    auto info = new LocationInfo;
    info->location = location;
    info->type = type;
    auto parentLocation = location->getParentLocationProxy();

    info->isRelativeLocation = false;
    if(info->type == LocationProxy::ParentRelativeLocation && parentLocation){
        info->isRelativeLocation = true;
    } else if(info->type == LocationProxy::OffsetLocation){
        info->isRelativeLocation = true;
    }
    info->isLocationAbleToBeGlobal = (!info->isRelativeLocation || parentLocation);

    info->isDependingOnAnotherTargetLocation = false;
        
    info->locationChangeConnection.reset(
        location->sigLocationChanged().connect(
            [this, info](){ onObjectLocationChanged(info); }));
    
    info->miscConnections.add(
        location->sigAttributeChanged().connect(
            [this, info](){
                if(currentCategory){
                    auto& infos = currentCategory->locationInfos;
                    if(infos.front() == info || infos[1] == info){
                        setupInterfaceForNewLocations();
                    }
                }
            }));

    info->miscConnections.add(
        location->sigExpired().connect(
            [this, info](){ onLocationExpired(info); }));

    if(ownerItem){
        info->miscConnections.add(
            ownerItem->sigDisconnectedFromRoot().connect(
                [this, info](){ onLocationExpired(info); }));
        info->item = ownerItem;
    }

    LocationCategoryPtr targetCategory;
    string categoryName = location->getCategory();
    for(auto& category : locationCategories){
        if(category->name == categoryName){
            targetCategory = category;
            break;
        }
    }
    if(!targetCategory){
        targetCategory = new LocationCategory;
        targetCategory->name = categoryName;
        locationCategories.push_back(targetCategory);
    }
    targetCategory->changed = true;
    targetCategory->locationInfos.push_back(info);
}


void LocationView::Impl::onObjectLocationChanged(LocationInfo* locationInfo)
{
    if(currentCategory){
        auto& locationInfos = currentCategory->locationInfos;
        if(locationInfo == locationInfos.front()){
            updatePositionWidgetWithPrimaryLocation();
        } else {
            for(auto& info : locationInfos){
                if(info == locationInfo){ // Included in the current category?
                    updatePrimaryRelativePosition(locationInfo);
                    break;
                }
            }
        }
    }
}


void LocationView::Impl::updatePrimaryRelativePosition(LocationInfo* locationInfo)
{
    if(locationInfo->type != LocationProxy::OffsetLocation){
        locationInfo->T_primary_relative =
            T_primary.inverse(Eigen::Isometry) * locationInfo->location->getGlobalLocation();
    }
}


void LocationView::Impl::onLocationExpired(LocationInfoPtr locationInfo)
{
    bool doUpdate = false;
    auto p = locationCategories.begin();
    while(p != locationCategories.end()){
        auto& category = *p;
        auto& infos = category->locationInfos;
        auto q = std::find(infos.begin(), infos.end(), locationInfo);
        if(q != infos.end()){
            infos.erase(q);
            bool isCategoryExpired = false;
            if(infos.empty()){
                locationCategories.erase(p);
                isCategoryExpired = true;
            }
            if(category == currentCategory){
                doUpdate = true;
                if(isCategoryExpired){
                    currentCategory.reset();
                }
            }
            break;
        }
        ++p;
    }
    managedLocations.erase(locationInfo->location);

    if(doUpdate){
        // Avoid redundant executions of the setupInterfaceForNewLocations function by using LazyCaller
        if(!setupInterfaceForNewLocationsLater.hasFunction()){
            setupInterfaceForNewLocationsLater.setFunction(
                [this](){ setupInterfaceForNewLocations(); });
        }
        setupInterfaceForNewLocationsLater();
    }
}
        

void LocationView::Impl::setupInterfaceForNewLocations()
{
    currentCategory.reset();
    
    if(locationCategories.empty()){
        singleCategoryLabel.setText("-----");
        singleCategoryLabel.show();
        categoryCombo.hide();
        individualRotationCheck.setEnabled(false);
        setEditorLocked(false, false);
        positionWidget->clearPosition();
        positionWidget->setEditable(false);
        clearBaseCoordinateSystems();

    } else {
        categoryCombo.blockSignals(true);
        categoryCombo.clear();
        
        int numCategories = locationCategories.size();
        singleCategoryLabel.setVisible(numCategories == 1);
        categoryCombo.setVisible(numCategories > 1);
        
        for(int i=0; i < numCategories; ++i){
            auto& category = locationCategories[i];
            auto& locationInfos = category->locationInfos;
            auto& location = locationInfos.front()->location;
            int n = locationInfos.size();
            if(numCategories == 1){
                if(n == 1){
                    singleCategoryLabel.setText(format(_("<b>{0}</b>"), location->getName()).c_str());
                } else if(n == 2){
                    singleCategoryLabel.setText(
                        format(_("<b>{0}</b> + {1}"), location->getName(), locationInfos[1]->location->getName()).c_str());
                } else {
                    singleCategoryLabel.setText(
                        format(_("<b>{0}</b> + {1} objects"),  location->getName(), n - 1).c_str());
                }
            } else {
                // Note that the default combo box implementation does not accept rich texts for its items and plain
                // texts are now used as follows. Rich texts may be used by introducing a custom delegate.
                if(n == 1){
                    categoryCombo.addItem(format(location->getName()).c_str());
                } else if(n == 2){
                    categoryCombo.addItem(
                        format(_("{0} + {1}"), location->getName(), locationInfos[1]->location->getName()).c_str());
                } else {
                    categoryCombo.addItem(
                        format(_("{0} + {1} objects"),  location->getName(), n - 1).c_str());
                }
            }

            // Update the dependency information
            for(auto& info : locationInfos){
                info->isDependingOnAnotherTargetLocation =
                    checkDependencyOnAnotherLocation(category, info->location->getParentLocationProxy());
            }
        }

        int categoryIndex = 0;
        for(size_t i=0; i < locationCategories.size(); ++i){
            if(locationCategories[i]->name == preferredCategoryName){
                categoryIndex = i;
                break;
            }
        }
        categoryCombo.setCurrentIndex(categoryIndex);
        
        categoryCombo.blockSignals(false);
        
        setCurrentLocationCategory(categoryIndex);
    }
}


bool LocationView::Impl::checkDependencyOnAnotherLocation(LocationCategory* category, LocationProxyPtr parentLocation)
{
    while(parentLocation){
        for(auto& info : category->locationInfos){
            if(info->location == parentLocation){
                return true;
            }
        }
        parentLocation = parentLocation->getParentLocationProxy();
    }
    return false;
}


void LocationView::Impl::setCurrentLocationCategory(int categoryIndex)
{
    currentCategory = locationCategories[categoryIndex];
    preferredCategoryName = currentCategory->name;

    auto& locationInfos = currentCategory->locationInfos;
    int numLocations = locationInfos.size();
    
    individualRotationCheck.setEnabled(numLocations >= 2);

    updateBaseCoordinateSystems();

    updatePositionWidgetWithPrimaryLocation();

    auto& primaryLocationInfo = locationInfos.front();
    bool locked = primaryLocationInfo->location->isLocked();
    if(numLocations >= 2){
        if(primaryLocationInfo->type == LocationProxy::OffsetLocation){
            locked = true;
        } else {
            for(int i = 1; i < numLocations; ++i){
                auto& subLocationInfo = locationInfos[i];
                if(subLocationInfo->type == LocationProxy::OffsetLocation ||
                   subLocationInfo->location->isLocked()){
                    locked = true;
                    break;
                }
                updatePrimaryRelativePosition(subLocationInfo);
            }
        }
    }
    bool blocked = locked;
    if(!blocked){
        for(auto& info : locationInfos){
            if(info->location->isDoingContinuousUpdate()){
                blocked = true;
                break;
            }
        }
    }
    
    setEditorLocked(locked, blocked);
}

    
void LocationView::Impl::setEditorLocked(bool locked, bool blocked)
{
    lockCheck.blockSignals(true);
    positionWidget->setEditable(!(locked || blocked));
    lockCheck.setChecked(locked);
    lockCheck.setEnabled(lockCheck.isChangable());
    lockCheck.blockSignals(false);
}


bool LockCheckBox::isChangable()
{
    if(!impl->currentCategory){
        return false;
    }
    auto& infos = impl->currentCategory->locationInfos;
    if(infos.empty()){
        return false;
    }
    int n = infos.size();
    bool changable = true;
    if(n >= 2){
        if(infos.front()->type == LocationProxy::OffsetLocation){
            changable = false;
        } else {
            for(int i=1; i < n; ++i){
                auto& info = infos[i];
                if(info->type == LocationProxy::OffsetLocation){
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
        for(auto& info : impl->currentCategory->locationInfos){
            info->miscConnections.block();
            info->location->setLocked(!isLocked);
            info->miscConnections.unblock();
        }
        impl->setupInterfaceForNewLocations();
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
    if(!currentCategory){
        return;
    }
    auto& locationInfo = currentCategory->locationInfos.front();
    auto location = locationInfo->location;
    
    clearBaseCoordinateSystems();
    int defaultComboIndex = 0;

    if(locationInfo->isLocationAbleToBeGlobal && (locationInfo->type != LocationProxy::OffsetLocation)){
        auto worldCoord = new CoordinateInfo(_("World"), BaseCoord, WorldCoordIndex);
        worldCoord->T.setIdentity();
        coordinates.push_back(worldCoord);
    }

    if(!locationInfo->isDependingOnAnotherTargetLocation){
        CoordinateInfo* parentCoord = nullptr;
        LocationProxyPtr parentLocation = location->getParentLocationProxy();
        if(parentLocation){
            string label;
            if(locationInfo->type == LocationProxy::OffsetLocation){
                label = format(_("Offset ( {} )"), parentLocation->getName());
            } else {
                label = format(_("Parent ( {} )"), parentLocation->getName());
            }
            parentCoord = new CoordinateInfo(label, ParentCoord, ParentCoordIndex);
            parentCoord->parentPositionFunc =
                [parentLocation](){ return parentLocation->getGlobalLocation(); };
            
        } else if(locationInfo->isRelativeLocation){
            if(locationInfo->type == LocationProxy::OffsetLocation){
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

    Isometry3 T = location->getLocation();
    auto localCoord = new CoordinateInfo(_("Local"), LocalCoord, LocalCoordIndex, T);
    localCoord->isLocal = true;
    localCoord->R0 = T.linear();
    coordinates.push_back(localCoord);

    if(locationInfo->isLocationAbleToBeGlobal && (locationInfo->type != LocationProxy::OffsetLocation)){

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

            if(checkDependencyOnAnotherLocation(currentCategory, frameParentLocation)){
                continue;
            }
            
            basename = frameParentLocation->getName();
            basename += " ";
            parentPositionFunc = [frameParentLocation](){ return frameParentLocation->getGlobalLocation(); };
            basename += frameListItem->displayName();
            basename += " ";
            auto frames = frameListItem->frameList();
            int n = frames->numFrames();

            Item* targetItem = locationInfo->item;
            if(!targetItem){
                targetItem = location->getCorrespondingItem();
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


void LocationView::Impl::setIndividualRotationMode(bool on)
{
    individualRotationCheck.blockSignals(true);
    individualRotationCheck.setChecked(on);
    individualRotationCheck.blockSignals(false);

    if(currentCategory){
        auto& locationInfos = currentCategory->locationInfos;
        if(!locationInfos.empty()){
            T_primary = locationInfos[0]->location->getGlobalLocation();
            for(size_t i = 1; i < locationInfos.size(); ++i){
                updatePrimaryRelativePosition(locationInfos[i]);
            }
        }
    }
}


void LocationView::Impl::updatePositionWidgetWithPrimaryLocation()
{
    if(!currentCategory){
        return;
    }

    auto& locationInfo = currentCategory->locationInfos.front();
    auto& location = locationInfo->location;

    auto& coord = coordinates[coordinateCombo.currentIndex()];
    lastCoordIndex = coord->index;
    Isometry3 T_display;

    if(coord->type == LocalCoord){
        Isometry3 T_location = location->getLocation();
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

        if(locationInfo->isRelativeLocation){
            if(coord->type == ParentCoord){
                T_display = location->getLocation();

            } else if(auto parentLocation = location->getParentLocationProxy()){
                Isometry3 T_parent = parentLocation->getGlobalLocation();
                T_location_global = T_parent * location->getLocation();
                isGlobalBase = true;
            } else {
                return; // invalid case
            }
        } else {
            T_location_global = location->getLocation();
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

    T_primary = location->getGlobalLocation();
}


bool LocationView::Impl::updateTargetLocationWithInputPosition(const Isometry3& T_input)
{
    if(!currentCategory){
        return false;
    }

    auto& locationInfos = currentCategory->locationInfos;
    auto& locationInfo = locationInfos.front();
    auto& location = locationInfo->location;

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
        if(locationInfo->isRelativeLocation && (coord->type == ParentCoord)){
            T_location = T_input;
        } else {
            Isometry3 T_base;
            if(coord->parentPositionFunc){
                T_base = coord->parentPositionFunc() * coord->T;
            } else {
                T_base.setIdentity();
            }
            if(locationInfo->isRelativeLocation){
                if(auto parentLocation = location->getParentLocationProxy()){
                    Isometry3 T_global = T_base * T_input;
                    Isometry3 T_parent = parentLocation->getGlobalLocation();
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

    for(auto& info : locationInfos){
        info->locationChangeConnection.block();
    }

    bool updated = false;

    // Synchronize the positions of the remaining location objects
    if(locationInfos.size() == 1){
        updated = location->setLocation(T_location);

    } else if(locationInfos.size() >= 2){
        Isometry3 T_global = location->getGlobalLocationOf(T_location);
        for(size_t i = 1; i < locationInfos.size(); ++i){
            auto& subLocation = locationInfos[i];
            if(!subLocation->isDependingOnAnotherTargetLocation){
                Isometry3 T;
                if(!individualRotationCheck.isChecked()){
                    T = T_global * subLocation->T_primary_relative;
                } else {
                    const auto& T_relative = subLocation->T_primary_relative;
                    T.linear() = T_global.linear() * T_relative.linear();
                    T.translation() = T_primary.linear() * T_relative.translation() + T_global.translation();
                }
                normalizeRotation(T);
                subLocation->location->setGlobalLocation(T);
            }
        }
        updated = location->setGlobalLocation(T_global);
    }

    for(auto& info : locationInfos){
        info->locationChangeConnection.unblock();
    }
    
    return updated;
}


void LocationView::Impl::finishLocationEditing()
{
    if(!currentCategory){
        return;
    }
    auto& locationInfos = currentCategory->locationInfos;
    
    if(locationInfos.size() == 1){
        locationInfos.front()->location->finishLocationEditing();

    } else if(locationInfos.size() >= 2){
        auto history = UnifiedEditHistory::instance();
        history->beginEditGroup(_("Change multiple object locations"));
        for(auto& info : locationInfos){
            info->location->finishLocationEditing();
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
    positionWidget->storeState(&archive);
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
    positionWidget->restoreState(&archive);

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
