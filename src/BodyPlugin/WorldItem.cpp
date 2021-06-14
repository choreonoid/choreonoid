/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldItem.h"
#include "MaterialTableItem.h"
#include "KinematicsBar.h"
#include <cnoid/ItemManager>
#include <cnoid/RootItem>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ConnectionSet>
#include <cnoid/LazyCaller>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/SceneCollision>
#include <cnoid/MaterialTable>
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace {

const bool TRACE_FUNCTIONS = false;

typedef CollisionDetector::GeometryHandle GeometryHandle;

struct ColdetLinkInfo : public Referenced
{
    BodyItem* bodyItem;
    Link* link;
    GeometryHandle geometry;
    ColdetLinkInfo(BodyItem* bodyItem, Link* link, GeometryHandle geometry)
        : bodyItem(bodyItem), link(link), geometry(geometry) { }
};
typedef ref_ptr<ColdetLinkInfo> ColdetLinkInfoPtr;
    
struct ColdetBodyInfo
{
    BodyItem* bodyItem;
    vector<ColdetLinkInfoPtr> linkInfos;
    LinkPtr parentBodyLink;
    bool kinematicStateChanged;
    bool isSelfCollisionDetectionEnabled;

    ColdetBodyInfo(BodyItem* bodyItem)
        : bodyItem(bodyItem),
          kinematicStateChanged(false)
    {
        isSelfCollisionDetectionEnabled = bodyItem->isSelfCollisionDetectionEnabled();
    }
};

}

namespace cnoid {

class WorldItem::Impl
{
public:
    WorldItem* self;
    ostream& os;
    KinematicsBar* kinematicsBar;

    ScopedConnectionSet sigKinematicStateChangedConnections;

    Selection collisionDetectorType;
    BodyCollisionDetector bodyCollisionDetector;
    vector<ColdetBodyInfo> coldetBodyInfos;
    std::shared_ptr<vector<CollisionLinkPairPtr>> collisions;
    Signal<void()> sigCollisionsUpdated;
    LazyCaller updateCollisionDetectorLater;
    LazyCaller updateCollisionsLater;
    SceneCollisionPtr sceneCollision;
    bool isCollisionDetectionEnabled;
    bool needToUpdateCollisionsLater;
    
    bool needToUpdateUnifiedMaterialTable;
    ItemList<MaterialTableItem> materialTableItems;
    MaterialTablePtr unifiedMaterialTable;
    MaterialTablePtr defaultMaterialTable;
    string defaultMaterialTableFile;
    std::time_t defaultMaterialTableTimestamp;

    Impl(WorldItem* self);
    Impl(WorldItem* self, const Impl& org);
    void init();
    ~Impl();
    void onSubTreeChanged();
    bool selectCollisionDetector(int index);
    void enableCollisionDetection(bool on);
    void clearCollisionDetector();
    void updateCollisionDetector(bool forceUpdate);
    void updateColdetBodyInfos(vector<ColdetBodyInfo>& infos);
    void updateCollisions(bool forceUpdate);
    void ignoreLinkPair(CollisionDetector* detector, GeometryHandle linkGeometry, Link* parentBodyLink, bool ignore);
    void extractCollisions(const CollisionPair& collisionPair);
    MaterialTable* getOrLoadDefaultMaterialTable(bool checkFileUpdate);
    MaterialTable* getOrCreateUnifiedMaterialTable();
};

}


void WorldItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<WorldItem>(N_("WorldItem"))
        .addCreationPanel<WorldItem>();
}


WorldItem::WorldItem()
{
    impl = new Impl(this);
    setName("World");
}


WorldItem::Impl::Impl(WorldItem* self)
    : self(self),
      os(mvout()),
      updateCollisionsLater([&](){ updateCollisions(false); }),
      updateCollisionDetectorLater([&](){ updateCollisionDetector(false); })
{
    const int n = CollisionDetector::numFactories();
    collisionDetectorType.resize(n);
    for(int i=0; i < n; ++i){
        collisionDetectorType.setSymbol(i, CollisionDetector::factoryName(i));
    };
    collisionDetectorType.select("AISTCollisionDetector");
    isCollisionDetectionEnabled = false;

    init();

    defaultMaterialTableFile = toUTF8((shareDirPath() / "default" / "materials.yaml").string());
}


WorldItem::WorldItem(const WorldItem& org)
    : Item(org)
{
    impl = new Impl(this, *org.impl);
}


WorldItem::Impl::Impl(WorldItem* self, const Impl& org)
    : self(self),
      os(org.os),
      updateCollisionsLater([&](){ updateCollisions(false); }),
      updateCollisionDetectorLater([&](){ updateCollisionDetector(false); }),
      defaultMaterialTableFile(org.defaultMaterialTableFile)
{
    collisionDetectorType = org.collisionDetectorType;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;

    init();
}


void WorldItem::Impl::init()
{
    kinematicsBar = KinematicsBar::instance();
    bodyCollisionDetector.setCollisionDetector(CollisionDetector::create(collisionDetectorType.selectedIndex()));
    bodyCollisionDetector.enableGeometryHandleMap(true);
    collisions = std::make_shared<vector<CollisionLinkPairPtr>>();
    sceneCollision = new SceneCollision(collisions);
    sceneCollision->setName("Collisions");
    defaultMaterialTableTimestamp = 0;
    needToUpdateCollisionsLater = false;
    needToUpdateUnifiedMaterialTable = true;

    self->sigSubTreeChanged().connect([&](){ onSubTreeChanged(); });
}

    
Item* WorldItem::doDuplicate() const
{
    return new WorldItem(*this);
}


WorldItem::~WorldItem()
{
    delete impl;
}


WorldItem::Impl::~Impl()
{

}


void WorldItem::Impl::onSubTreeChanged()
{
    if(isCollisionDetectionEnabled){
        updateCollisionDetectorLater();
    }
    needToUpdateUnifiedMaterialTable = true;
}


void WorldItem::storeCurrentBodyPositionsAsInitialPositions()
{
    for(auto& bodyItem : descendantItems<BodyItem>()){
        bodyItem->storeInitialState();
    }
}


void WorldItem::restoreInitialBodyPositions(bool doNotify)
{
    for(auto& bodyItem : descendantItems<BodyItem>()){
        bodyItem->restoreInitialState(doNotify);
    }
}


ItemList<BodyItem> WorldItem::coldetBodyItems() const
{
    ItemList<BodyItem> bodyItems;
    for(auto& info : impl->coldetBodyInfos){
        bodyItems.push_back(info.bodyItem);
    }
    return bodyItems;
}


std::vector<CollisionLinkPairPtr>& WorldItem::collisions() const
{
    return *impl->collisions;
}


bool WorldItem::selectCollisionDetector(const std::string& name)
{
    return impl->selectCollisionDetector(CollisionDetector::factoryIndex(name));
}


bool WorldItem::Impl::selectCollisionDetector(int index)
{
    if(index >= 0 && index < collisionDetectorType.size()){
        CollisionDetector* newCollisionDetector = CollisionDetector::create(index);
        if(newCollisionDetector){
            bodyCollisionDetector.setCollisionDetector(newCollisionDetector);
            collisionDetectorType.select(index);
            if(isCollisionDetectionEnabled){
                updateCollisionDetector(true);
            }
            return true;
        }
    }
    return false;
}


CollisionDetector* WorldItem::collisionDetector()
{
    return impl->bodyCollisionDetector.collisionDetector();
}


void WorldItem::setCollisionDetectionEnabled(bool on)
{
    impl->enableCollisionDetection(on);
}


void WorldItem::Impl::enableCollisionDetection(bool on)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItem::Impl::enableCollisionDetection(" << on << ")" << endl;
    }

    bool changed = false;
    
    if(isCollisionDetectionEnabled && !on){
        clearCollisionDetector();
        isCollisionDetectionEnabled = false;
        changed = true;
        
    } else if(!isCollisionDetectionEnabled && on){
        isCollisionDetectionEnabled = true;
        updateCollisionDetector(true);
        changed = true;
    }

    if(changed){
        self->notifyUpdate();
        sigCollisionsUpdated();
    }
}


bool WorldItem::isCollisionDetectionEnabled()
{
    return impl->isCollisionDetectionEnabled;
}


void WorldItem::Impl::clearCollisionDetector()
{
    if(TRACE_FUNCTIONS){
        os << "WorldItem::Impl::clearCollisionDetector()" << endl;
    }

    bodyCollisionDetector.clearBodies();
    sigKinematicStateChangedConnections.disconnect();
    updateCollisionsLater.cancel();
    needToUpdateCollisionsLater = false;

    for(auto& info : coldetBodyInfos){
        info.bodyItem->clearCollisions();
    }
}


void WorldItem::updateCollisionDetectorLater()
{
    impl->updateCollisionDetectorLater();
}


void WorldItem::updateCollisionDetector()
{
    impl->updateCollisionDetector(true);
}


void WorldItem::Impl::updateCollisionDetector(bool forceUpdate)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItem::Impl::updateCollisionDetector()" << endl;
    }

    if(!isCollisionDetectionEnabled){
        return;
    }

    if(forceUpdate){
        updateColdetBodyInfos(coldetBodyInfos);
    } else {
        vector<ColdetBodyInfo> infos;
        updateColdetBodyInfos(infos);
        if(infos.size() == coldetBodyInfos.size()){
            if(std::equal(infos.begin(), infos.end(), coldetBodyInfos.begin(),
                          [](ColdetBodyInfo& info1, ColdetBodyInfo& info2){
                              return (info1.bodyItem == info2.bodyItem &&
                                      info1.isSelfCollisionDetectionEnabled == info2.isSelfCollisionDetectionEnabled); })){
                // A set of body items are not changed
                if(needToUpdateCollisionsLater){
                    needToUpdateCollisionsLater = false;
                    updateCollisions(false);
                    return;
                }
            }
        }
        coldetBodyInfos = infos;
    }

    clearCollisionDetector();

    for(auto& bodyInfo : coldetBodyInfos){
        BodyItem* bodyItem = bodyInfo.bodyItem;

        bodyCollisionDetector.addBody(
            bodyItem->body(), bodyInfo.isSelfCollisionDetectionEnabled,
            [&bodyInfo](Link* link, GeometryHandle geometry){
                ColdetLinkInfo* linkInfo = new ColdetLinkInfo(bodyInfo.bodyItem, link, geometry);
                bodyInfo.linkInfos.push_back(linkInfo);
                return linkInfo;
            });

        ColdetBodyInfo* pBodyInfo = &bodyInfo;
        sigKinematicStateChangedConnections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [&, pBodyInfo](){
                    pBodyInfo->kinematicStateChanged = true;
                    updateCollisionsLater.setPriority(kinematicsBar->collisionDetectionPriority());
                    updateCollisionsLater();
                }));
    }

    bodyCollisionDetector.makeReady();
    updateCollisions(true);
}


void WorldItem::Impl::updateColdetBodyInfos(vector<ColdetBodyInfo>& infos)
{
    infos.clear();
    for(auto bodyItem : self->descendantItems<BodyItem>()){
        if(bodyItem->isCollisionDetectionEnabled()){
            infos.push_back(ColdetBodyInfo(bodyItem));
        }
    }
}


void WorldItem::updateCollisions()
{
    impl->updateCollisions(true);
}


void WorldItem::Impl::updateCollisions(bool forceUpdate)
{
    if(updateCollisionDetectorLater.isPending()){
        needToUpdateCollisionsLater = true;
        return;
    }
    
    auto collisionDetector = bodyCollisionDetector.collisionDetector();

    for(auto& bodyInfo : coldetBodyInfos){
        auto bodyItem = bodyInfo.bodyItem;
        bodyItem->clearCollisions();
        if(bodyInfo.kinematicStateChanged || forceUpdate){
            for(auto& linkInfo : bodyInfo.linkInfos){
                collisionDetector->updatePosition(
                    linkInfo->geometry, linkInfo->link->position());
            }
        }
        Link* prevParentBodyLink = bodyInfo.parentBodyLink;
        Link* newParentBodyLink = nullptr;
        if(bodyItem->isAttachedToParentBody()){
            newParentBodyLink = bodyItem->body()->parentBodyLink();
        }
        if(newParentBodyLink != prevParentBodyLink){
            auto rootLinkGeometry = bodyInfo.linkInfos[0]->geometry;
            if(prevParentBodyLink){
                ignoreLinkPair(collisionDetector, rootLinkGeometry, prevParentBodyLink, false);
            }
            if(newParentBodyLink){
                ignoreLinkPair(collisionDetector, rootLinkGeometry, newParentBodyLink, true);
            }
            bodyInfo.parentBodyLink = newParentBodyLink;
        }
        
        bodyInfo.kinematicStateChanged = false;
    }
    
    collisions->clear();
    
    bodyCollisionDetector.detectCollisions(
        [&](const CollisionPair& pair){ extractCollisions(pair); });
    
    sceneCollision->setDirty();
    
    for(auto& info : coldetBodyInfos){
        info.bodyItem->notifyCollisionUpdate();
    }
    
    sigCollisionsUpdated();
}


void WorldItem::Impl::ignoreLinkPair
(CollisionDetector* detector, GeometryHandle linkGeometry, Link* parentBodyLink, bool ignore)
{
    while(parentBodyLink){
        if(auto pParentLinkGeometry = bodyCollisionDetector.findGeometryHandle(parentBodyLink)){
            detector->ignoreGeometryPair(linkGeometry, *pParentLinkGeometry, ignore);
        }
        if(parentBodyLink->isFixedJoint()){
            parentBodyLink = parentBodyLink->parent();
        } else {
            break;
        }
    }
}


void WorldItem::Impl::extractCollisions(const CollisionPair& collisionPair)
{
    CollisionLinkPairPtr collisionLinkPair = std::make_shared<CollisionLinkPair>();
    collisionLinkPair->collisions = collisionPair.collisions();
    BodyItem* bodyItem = 0;
    for(int i=0; i < 2; ++i){
        ColdetLinkInfo* linkInfo = static_cast<ColdetLinkInfo*>(collisionPair.object(i));
        if(linkInfo->bodyItem != bodyItem){
            bodyItem = linkInfo->bodyItem;
            bodyItem->collisions().push_back(collisionLinkPair);
        }
        collisionLinkPair->body[i] = bodyItem->body();
        Link* link = linkInfo->link;
        collisionLinkPair->link[i] = link;
        bodyItem->collisionsOfLink(link->index()).push_back(collisionLinkPair);
        bodyItem->collisionLinkBitSet()[link->index()] = true;
    }
    collisions->push_back(collisionLinkPair);
}


SignalProxy<void()> WorldItem::sigCollisionsUpdated()
{
    return impl->sigCollisionsUpdated;
}


void WorldItem::setDefaultMaterialTableFile(const std::string& filename)
{
    if(filename != impl->defaultMaterialTableFile){
        impl->defaultMaterialTable = nullptr;
        impl->defaultMaterialTableFile = filename;
    }
}


MaterialTable* WorldItem::defaultMaterialTable(bool checkFileUpdate)
{
    return impl->getOrLoadDefaultMaterialTable(checkFileUpdate);
}


MaterialTable* WorldItem::Impl::getOrLoadDefaultMaterialTable(bool checkFileUpdate)
{
    bool failedToLoad = false;
    
    if(!defaultMaterialTable){
        defaultMaterialTable = new MaterialTable;
        if(defaultMaterialTable->load(defaultMaterialTableFile, os)){
            defaultMaterialTableTimestamp =
                filesystem::last_write_time_to_time_t(fromUTF8(defaultMaterialTableFile));
        } else {
            failedToLoad = true;
        }

    } else if(checkFileUpdate){
        if(!defaultMaterialTableFile.empty()){
            filesystem::path fpath(fromUTF8(defaultMaterialTableFile));
            if(filesystem::exists(fpath)){
                auto newTimestamp = filesystem::last_write_time_to_time_t(fpath);
                if(newTimestamp > defaultMaterialTableTimestamp){
                    MaterialTablePtr newMaterialTable = new MaterialTable;
                    if(newMaterialTable->load(defaultMaterialTableFile, os)){
                        defaultMaterialTable = newMaterialTable;
                        defaultMaterialTableTimestamp = newTimestamp;
                        MessageView::instance()->putln(
                            format(_("Default material table \"{}\" has been reloaded."), defaultMaterialTableFile));
                    } else {
                        failedToLoad = true;
                    }
                }
            }
        }
    }

    if(failedToLoad){
        MessageView::instance()->putln(
            format(_("Reloading default material table \"{}\" failed."), defaultMaterialTableFile),
            MessageView::Warning);
    }

    return defaultMaterialTable;
}


MaterialTable* WorldItem::materialTable()
{
    return impl->getOrCreateUnifiedMaterialTable();
}


MaterialTable* WorldItem::Impl::getOrCreateUnifiedMaterialTable()
{
    if(!needToUpdateUnifiedMaterialTable){
        return unifiedMaterialTable;
    }
    
    auto newMaterialTableItems = self->descendantItems<MaterialTableItem>();

    if(newMaterialTableItems.empty()){
        unifiedMaterialTable = getOrLoadDefaultMaterialTable(false);
        materialTableItems.clear();

    } else if(newMaterialTableItems != materialTableItems){
        unifiedMaterialTable = new MaterialTable(*getOrLoadDefaultMaterialTable(false));
        for(auto& materialTableItem : newMaterialTableItems){
            unifiedMaterialTable->merge(materialTableItem->materialTable());
        }
        materialTableItems.swap(newMaterialTableItems);
    }

    needToUpdateUnifiedMaterialTable = false;
    return unifiedMaterialTable;
}


SgNode* WorldItem::getScene()
{
    return impl->sceneCollision;
}


void WorldItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Collision detection"), isCollisionDetectionEnabled(),
                [&](bool on){ setCollisionDetectionEnabled(on); return true; });
    putProperty(_("Collision detector"), impl->collisionDetectorType,
                [&](int index){ return impl->selectCollisionDetector(index); });

    FilePathProperty materialFileProperty(
        impl->defaultMaterialTableFile, { _("Contact material definition file (*.yaml)") });
    putProperty(_("Default material table"), materialFileProperty,
                [&](const string& filename){ setDefaultMaterialTableFile(filename); return true; });
}


bool WorldItem::store(Archive& archive)
{
    archive.write("collision_detection", isCollisionDetectionEnabled());
    archive.write("collision_detector", impl->collisionDetectorType.selectedSymbol());
    archive.writeRelocatablePath("default_material_table_file", impl->defaultMaterialTableFile);
    return true;
}


bool WorldItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read({ "collision_detector", "collisionDetector" }, symbol)){
        selectCollisionDetector(symbol);
    }
    if(archive.get({ "collision_detection", "collisionDetection" }, false)){
        archive.addPostProcess([&](){ impl->enableCollisionDetection(true); });
    }
    if(archive.read({ "default_material_table_file", "materialTableFile" }, symbol)){
        symbol = archive.resolveRelocatablePath(symbol);
        if(!symbol.empty()){
            setDefaultMaterialTableFile(symbol);
        }
    }
    return true;
}
