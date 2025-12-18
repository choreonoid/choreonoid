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
#include <cnoid/Format>
#include <filesystem>
#include <unordered_map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

typedef CollisionDetector::GeometryHandle GeometryHandle;

struct ColdetBodyInfo : public Referenced
{
    BodyItem* bodyItem;
    LinkPtr parentBodyLink;
    bool isSelfCollisionDetectionEnabled;
    bool isModelUpdated;

    ColdetBodyInfo(BodyItem* bodyItem)
        : bodyItem(bodyItem),
          isModelUpdated(false)
    {
        isSelfCollisionDetectionEnabled = bodyItem->isSelfCollisionDetectionEnabled();
    }
};
typedef ref_ptr<ColdetBodyInfo> ColdetBodyInfoPtr;

}

namespace cnoid {

class WorldItem::Impl
{
public:
    WorldItem* self;
    ostream& os;
    KinematicsBar* kinematicsBar;

    ScopedConnectionSet bodyItemConnections;

    Selection collisionDetectorType;
    BodyCollisionDetector bodyCollisionDetector;
    vector<ColdetBodyInfoPtr> coldetBodyInfos;
    unordered_map<BodyPtr, BodyItem*> bodyToBodyItemMap;
    std::shared_ptr<vector<CollisionLinkPairPtr>> collisions;
    Signal<void()> sigCollisionsUpdated;
    LazyCaller updateCollisionDetectionBodiesLater;
    LazyCaller updateCollisionsLater;
    SceneCollisionPtr sceneCollision;
    bool isCollisionDetectionEnabled;
    bool isCollisionDetectionBetweenMultiplexBodiesEnabled;
    bool needToUpdateCollisionsLater;
    
    bool needToUpdateUnifiedMaterialTable;
    ItemList<MaterialTableItem> materialTableItems;
    MaterialTablePtr unifiedMaterialTable;
    MaterialTablePtr defaultMaterialTable;
    string defaultMaterialTableFile;
    std::filesystem::file_time_type defaultMaterialTableTimestamp;

    Impl(WorldItem* self);
    Impl(WorldItem* self, const Impl& org);
    void init();
    ~Impl();
    void onSubTreeChanged();
    bool selectCollisionDetector(int index, bool doUpdateCollisionDetectionBodies);
    void enableCollisionDetection(bool on);
    void clearCollisionDetector(bool doNotiyCollisionUpdate);
    void updateCollisionDetectionBodies(bool forceUpdate);
    void updateColdetBodyInfos(vector<ColdetBodyInfoPtr>& infos);
    void updateCollisions(bool forceUpdate);
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
      updateCollisionDetectionBodiesLater([&](){ updateCollisionDetectionBodies(false); })
{
    const int n = CollisionDetector::numFactories();
    collisionDetectorType.resize(n);
    for(int i=0; i < n; ++i){
        collisionDetectorType.setSymbol(i, CollisionDetector::factoryName(i));
    };
    collisionDetectorType.select("AISTCollisionDetector");
    isCollisionDetectionEnabled = false;
    isCollisionDetectionBetweenMultiplexBodiesEnabled = false;

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
      updateCollisionDetectionBodiesLater([&](){ updateCollisionDetectionBodies(false); }),
      defaultMaterialTableFile(org.defaultMaterialTableFile)
{
    collisionDetectorType = org.collisionDetectorType;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;
    isCollisionDetectionBetweenMultiplexBodiesEnabled = org.isCollisionDetectionBetweenMultiplexBodiesEnabled;

    init();
}


void WorldItem::Impl::init()
{
    kinematicsBar = KinematicsBar::instance();
    collisions = std::make_shared<vector<CollisionLinkPairPtr>>();
    sceneCollision = new SceneCollision(collisions);
    sceneCollision->setAttribute(SgObject::MetaScene);
    sceneCollision->setName("Collisions");
    defaultMaterialTableTimestamp = {};
    needToUpdateCollisionsLater = false;
    needToUpdateUnifiedMaterialTable = true;

    selectCollisionDetector(collisionDetectorType.selectedIndex(), false);
    
    self->sigSubTreeChanged().connect([&](){ onSubTreeChanged(); });
}

    
Item* WorldItem::doCloneItem(CloneMap* /* cloneMap */) const
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
        updateCollisionDetectionBodiesLater();
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
        bodyItems.push_back(info->bodyItem);
    }
    return bodyItems;
}


std::vector<WorldItem::CollisionLinkPairPtr>& WorldItem::collisions() const
{
    return *impl->collisions;
}


bool WorldItem::selectCollisionDetector(const std::string& name)
{
    return impl->selectCollisionDetector(CollisionDetector::factoryIndex(name), true);
}


bool WorldItem::Impl::selectCollisionDetector(int index, bool doUpdateCollisionDetectionBodies)
{
    if(index >= 0 && index < collisionDetectorType.size()){
        CollisionDetector* newCollisionDetector = CollisionDetector::create(index);
        if(newCollisionDetector){
            bodyCollisionDetector.setCollisionDetector(newCollisionDetector);
            bodyCollisionDetector.setGeometryHandleMapEnabled(true);

            collisionDetectorType.select(index);
            if(doUpdateCollisionDetectionBodies && isCollisionDetectionEnabled){
                updateCollisionDetectionBodies(true);
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
    bool changed = false;
    
    if(isCollisionDetectionEnabled && !on){
        clearCollisionDetector(true);
        isCollisionDetectionEnabled = false;
        changed = true;
        
    } else if(!isCollisionDetectionEnabled && on){
        isCollisionDetectionEnabled = true;
        updateCollisionDetectionBodies(true);
        changed = true;
    }

    if(changed){
        self->notifyUpdate();
    }
}


bool WorldItem::isCollisionDetectionEnabled()
{
    return impl->isCollisionDetectionEnabled;
}


void WorldItem::setCollisionDetectionBetweenMultiplexBodiesEnabled(bool on)
{
    impl->isCollisionDetectionBetweenMultiplexBodiesEnabled = on;
}


bool WorldItem::isCollisionDetectionBetweenMultiplexBodiesEnabled() const
{
    return impl->isCollisionDetectionBetweenMultiplexBodiesEnabled;
}


void WorldItem::Impl::clearCollisionDetector(bool doNotiyCollisionUpdate)
{
    collisions->clear();
    sceneCollision->setDirty();
    bodyCollisionDetector.clearBodies();
    bodyItemConnections.disconnect();
    updateCollisionsLater.cancel();
    needToUpdateCollisionsLater = false;

    for(auto& info : coldetBodyInfos){
        info->bodyItem->clearCollisions(doNotiyCollisionUpdate);
    }

    if(doNotiyCollisionUpdate){
        sigCollisionsUpdated();
    }
}


void WorldItem::updateCollisionDetectionBodies()
{
    impl->updateCollisionDetectionBodies(true);
}


void WorldItem::Impl::updateCollisionDetectionBodies(bool forceUpdate)
{
    if(!isCollisionDetectionEnabled){
        return;
    }

    if(forceUpdate){
        updateColdetBodyInfos(coldetBodyInfos);
    } else {
        vector<ColdetBodyInfoPtr> infos;
        updateColdetBodyInfos(infos);
        if(infos.size() == coldetBodyInfos.size()){
            bool unchanged = std::equal(
                infos.begin(), infos.end(), coldetBodyInfos.begin(),
                [](ColdetBodyInfoPtr& info1, ColdetBodyInfoPtr& info2){
                    return (info1->bodyItem == info2->bodyItem &&
                            info1->isSelfCollisionDetectionEnabled == info2->isSelfCollisionDetectionEnabled &&
                            !info2->isModelUpdated);
                });
            if(unchanged){
                // A set of body items are not changed
                if(needToUpdateCollisionsLater){
                    needToUpdateCollisionsLater = false;
                    updateCollisions(false);
                }
                return;
            }
        }
        coldetBodyInfos.swap(infos);
    }

    bodyToBodyItemMap.clear();
    for(auto& info : coldetBodyInfos){
        bodyToBodyItemMap[info->bodyItem->body()] = info->bodyItem;
    }

    clearCollisionDetector(false);

    bodyCollisionDetector.setMultiplexBodySupportEnabled(isCollisionDetectionBetweenMultiplexBodiesEnabled);

    for(auto& bodyInfo : coldetBodyInfos){
        BodyItem* bodyItem = bodyInfo->bodyItem;

        bodyCollisionDetector.addBody(bodyItem->body(), bodyInfo->isSelfCollisionDetectionEnabled);

        bodyItemConnections.add(
            bodyItem->sigKinematicStateChanged().connect(
                [this, bodyInfo](){
                    updateCollisionsLater.setPriority(kinematicsBar->collisionDetectionPriority());
                    updateCollisionsLater();
                }));
        bodyItemConnections.add(
            bodyItem->sigModelUpdated().connect(
                [this, bodyInfo](int flags){
                    if(flags & (BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate | BodyItem::ShapeUpdate)){
                        bodyInfo->isModelUpdated = true;
                        updateCollisionDetectionBodiesLater();
                    }
                }));
    }

    updateCollisions(true);
}


void WorldItem::Impl::updateColdetBodyInfos(vector<ColdetBodyInfoPtr>& infos)
{
    infos.clear();
    for(auto bodyItem : self->descendantItems<BodyItem>()){
        if(bodyItem->isCollisionDetectionEnabled()){
            infos.push_back(new ColdetBodyInfo(bodyItem));
        }
    }
}


void WorldItem::updateCollisionDetectionBodiesLater()
{
    impl->updateCollisionDetectionBodiesLater();
}


void WorldItem::updateCollisions()
{
    impl->updateCollisions(true);
}


void WorldItem::Impl::updateCollisions(bool forceUpdate)
{
    if(updateCollisionDetectionBodiesLater.isPending()){
        needToUpdateCollisionsLater = true;
        return;
    }

    bodyCollisionDetector.makeReady();
    bodyCollisionDetector.updatePositions();
    
    auto collisionDetector = bodyCollisionDetector.collisionDetector();

    for(auto& bodyInfo : coldetBodyInfos){
        auto bodyItem = bodyInfo->bodyItem;
        bodyItem->clearCollisions(false);

        auto body = bodyItem->body();
        Link* newParentBodyLink = bodyItem->isAttachedToParentBody() ? body->parentBodyLink() : nullptr;
        Link* prevParentBodyLink = bodyInfo->parentBodyLink;
        if(newParentBodyLink != prevParentBodyLink){
            auto rootLink = body->rootLink();
            if(prevParentBodyLink){
                bodyCollisionDetector.setLinksInAttachmentIgnored(rootLink, prevParentBodyLink, false);
            }
            if(newParentBodyLink){
                bodyCollisionDetector.setLinksInAttachmentIgnored(rootLink, newParentBodyLink, true);
            }
            bodyInfo->parentBodyLink = newParentBodyLink;
        }
    }
    
    collisions->clear();
    
    bodyCollisionDetector.detectCollisions(
        [&](const CollisionPair& pair){
            extractCollisions(pair);
            return false; // Continue checking all collisions
        });
    
    sceneCollision->setDirty();
    
    for(auto& info : coldetBodyInfos){
        info->bodyItem->notifyCollisionUpdate();
    }
    
    sigCollisionsUpdated();
}


void WorldItem::Impl::extractCollisions(const CollisionPair& collisionPair)
{
    CollisionLinkPairPtr collisionLinkPair = std::make_shared<CollisionLinkPair>();
    collisionLinkPair->setCollisions(collisionPair.collisions());
    BodyItem* anotherBodyItem = nullptr;
    for(int i=0; i < 2; ++i){
        auto link = static_cast<Link*>(collisionPair.object(i));
        auto mainBody = link->body()->multiplexMainBody();
        auto it = bodyToBodyItemMap.find(mainBody);
        if(it != bodyToBodyItemMap.end()){
            auto bodyItem = it->second;
            if(bodyItem != anotherBodyItem){
                bodyItem->collisions().push_back(collisionLinkPair);
                anotherBodyItem = bodyItem;
            }
            bodyItem->collisionsOfLink(link->index()).push_back(collisionLinkPair);
            bodyItem->collisionLinkBitSet()[link->index()] = true;
        }
        collisionLinkPair->setLink(i, link);
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
                filesystem::last_write_time(fromUTF8(defaultMaterialTableFile));
        } else {
            failedToLoad = true;
        }

    } else if(checkFileUpdate){
        if(!defaultMaterialTableFile.empty()){
            filesystem::path fpath(fromUTF8(defaultMaterialTableFile));
            if(filesystem::exists(fpath)){
                auto newTimestamp = filesystem::last_write_time(fpath);
                if(newTimestamp > defaultMaterialTableTimestamp){
                    MaterialTablePtr newMaterialTable = new MaterialTable;
                    if(newMaterialTable->load(defaultMaterialTableFile, os)){
                        defaultMaterialTable = newMaterialTable;
                        defaultMaterialTableTimestamp = newTimestamp;
                        MessageView::instance()->putln(
                            formatR(_("Default material table \"{}\" has been reloaded."), defaultMaterialTableFile));
                    } else {
                        failedToLoad = true;
                    }
                }
            }
        }
    }

    if(failedToLoad){
        MessageView::instance()->putln(
            formatR(_("Reloading default material table \"{}\" failed."), defaultMaterialTableFile),
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
                [this](bool on){ setCollisionDetectionEnabled(on); return true; });
    putProperty(_("Collision detector"), impl->collisionDetectorType,
                [this](int index){ return impl->selectCollisionDetector(index, true); });

    putProperty(_("Collision detection between multiplex bodies"), impl->isCollisionDetectionBetweenMultiplexBodiesEnabled,
                [this](bool on){
                    setCollisionDetectionBetweenMultiplexBodiesEnabled(on);
                    impl->updateCollisionDetectionBodies(true);
                    return true;
                });
    
    FilePathProperty materialFileProperty(
        impl->defaultMaterialTableFile, { _("Contact material definition file (*.yaml)") });
    putProperty(_("Default material table"), materialFileProperty,
                [this](const string& filename){ setDefaultMaterialTableFile(filename); return true; });
}


bool WorldItem::store(Archive& archive)
{
    archive.write("collision_detection", isCollisionDetectionEnabled());
    archive.write("collision_detector", impl->collisionDetectorType.selectedSymbol());
    if(impl->isCollisionDetectionBetweenMultiplexBodiesEnabled){
        archive.write("collision_detection_between_multiplex_bodies", true);
    }
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
    archive.read("collision_detection_between_multiplex_bodies", impl->isCollisionDetectionBetweenMultiplexBodiesEnabled);
    
    if(archive.read({ "default_material_table_file", "materialTableFile" }, symbol)){
        symbol = archive.resolveRelocatablePath(symbol);
        if(!symbol.empty()){
            setDefaultMaterialTableFile(symbol);
        }
    }
    
    return true;
}
