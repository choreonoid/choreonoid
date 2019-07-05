/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldItem.h"
#include "KinematicsBar.h"
#include <cnoid/ItemManager>
#include <cnoid/RootItem>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/ConnectionSet>
#include <cnoid/LazyCaller>
#include <cnoid/BodyCollisionDetector>
#include <cnoid/SceneCollision>
#include <cnoid/MaterialTable>
#include <cnoid/ExecutablePath>
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

class WorldItemImpl
{
public:
    WorldItem* self;
    ostream& os;
    KinematicsBar* kinematicsBar;

    Connection sigSubTreeChangedConnection;
    ConnectionSet sigKinematicStateChangedConnections;

    Selection collisionDetectorType;
    BodyCollisionDetector bodyCollisionDetector;
    vector<ColdetBodyInfo> coldetBodyInfos;
    std::shared_ptr<vector<CollisionLinkPairPtr>> collisions;
    Signal<void()> sigCollisionsUpdated;
    LazyCaller updateCollisionDetectorLater;
    LazyCaller updateCollisionsLater;
    bool isCollisionDetectionEnabled;
    SceneCollisionPtr sceneCollision;

    string materialTableFile;
    MaterialTablePtr materialTable;
    std::time_t materialTableTimestamp;

    WorldItemImpl(WorldItem* self);
    WorldItemImpl(WorldItem* self, const WorldItemImpl& org);
    ~WorldItemImpl();
    void init();
    bool selectCollisionDetector(int index);
    void enableCollisionDetection(bool on);
    void clearCollisionDetector();
    void updateCollisionDetector(bool forceUpdate);
    void updateColdetBodyInfos(vector<ColdetBodyInfo>& infos);
    void updateCollisions(bool forceUpdate);
    void extractCollisions(const CollisionPair& collisionPair);
    MaterialTable* getOrLoadMaterialTable(bool checkFileUpdate);
};

}


void WorldItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<WorldItem>(N_("WorldItem"));
    ext->itemManager().addCreationPanel<WorldItem>();
}


WorldItem::WorldItem()
{
    impl = new WorldItemImpl(this);
    setName("World");
}


WorldItemImpl::WorldItemImpl(WorldItem* self)
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

    materialTableFile = (filesystem::path(shareDirectory()) / "default" / "materials.yaml").string();
}


WorldItem::WorldItem(const WorldItem& org)
    : Item(org)
{
    impl = new WorldItemImpl(this, *org.impl);
}


WorldItemImpl::WorldItemImpl(WorldItem* self, const WorldItemImpl& org)
    : self(self),
      os(org.os),
      updateCollisionsLater([&](){ updateCollisions(false); }),
      updateCollisionDetectorLater([&](){ updateCollisionDetector(false); }),
      materialTableFile(org.materialTableFile)
{
    collisionDetectorType = org.collisionDetectorType;
    isCollisionDetectionEnabled = org.isCollisionDetectionEnabled;

    init();
}


void WorldItemImpl::init()
{
    kinematicsBar = KinematicsBar::instance();
    bodyCollisionDetector.setCollisionDetector(CollisionDetector::create(collisionDetectorType.selectedIndex()));
    collisions = std::make_shared<vector<CollisionLinkPairPtr>>();
    sceneCollision = new SceneCollision(collisions);
    sceneCollision->setName("Collisions");
    materialTableTimestamp = 0;
}

    
WorldItem::~WorldItem()
{
    delete impl;
}


WorldItemImpl::~WorldItemImpl()
{
    sigKinematicStateChangedConnections.disconnect();
    sigSubTreeChangedConnection.disconnect();
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


bool WorldItemImpl::selectCollisionDetector(int index)
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


void WorldItem::enableCollisionDetection(bool on)
{
    impl->enableCollisionDetection(on);
}


void WorldItemImpl::enableCollisionDetection(bool on)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::enableCollisionDetection(" << on << ")" << endl;
    }

    bool changed = false;
    
    if(isCollisionDetectionEnabled && !on){
        clearCollisionDetector();
        sigSubTreeChangedConnection.disconnect();
        isCollisionDetectionEnabled = false;
        changed = true;
        
    } else if(!isCollisionDetectionEnabled && on){
        isCollisionDetectionEnabled = true;

        updateCollisionDetector(true);
        sigSubTreeChangedConnection =
            self->sigSubTreeChanged().connect(
                [&](){ self->updateCollisionDetector(); });
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


void WorldItemImpl::clearCollisionDetector()
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::clearCollisionDetector()" << endl;
    }

    bodyCollisionDetector.clearBodies();
    sigKinematicStateChangedConnections.disconnect();

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


/**
   \todo reduce the number of calling this function when the project is loaded.
*/
void WorldItemImpl::updateCollisionDetector(bool forceUpdate)
{
    if(TRACE_FUNCTIONS){
        os << "WorldItemImpl::updateCollisionDetector()" << endl;
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
                return; // not changed
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


void WorldItemImpl::updateColdetBodyInfos(vector<ColdetBodyInfo>& infos)
{
    infos.clear();
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(self);
    for(auto bodyItem : bodyItems){
        if(bodyItem->isCollisionDetectionEnabled()){
            infos.push_back(ColdetBodyInfo(bodyItem));
        }
    }
}


void WorldItem::updateCollisions()
{
    impl->updateCollisions(true);
}


void WorldItemImpl::updateCollisions(bool forceUpdate)
{
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


void WorldItemImpl::extractCollisions(const CollisionPair& collisionPair)
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


SgNode* WorldItem::getScene()
{
    return impl->sceneCollision;
}


void WorldItem::setMaterialTableFile(const std::string& filename)
{
    if(filename != impl->materialTableFile){
        impl->materialTable = nullptr;
        impl->materialTableFile = filename;
    }
}


MaterialTable* WorldItem::materialTable(bool checkFileUpdate)
{
    return impl->getOrLoadMaterialTable(checkFileUpdate);
}


MaterialTable* WorldItemImpl::getOrLoadMaterialTable(bool checkFileUpdate)
{
    bool failedToLoad = false;
    
    if(!materialTable){
        materialTable = new MaterialTable;
        if(materialTable->load(materialTableFile, os)){
            materialTableTimestamp = filesystem::last_write_time_to_time_t(materialTableFile);
        } else {
            failedToLoad = true;
        }

    } else if(checkFileUpdate){
        if(!materialTableFile.empty()){
            filesystem::path fpath(materialTableFile);
            if(filesystem::exists(fpath)){
                auto newTimestamp = filesystem::last_write_time_to_time_t(materialTableFile);
                if(newTimestamp > materialTableTimestamp){
                    MaterialTablePtr newMaterialTable = new MaterialTable;
                    if(newMaterialTable->load(materialTableFile, os)){
                        materialTable = newMaterialTable;
                        materialTableTimestamp = newTimestamp;
                        MessageView::instance()->putln(
                            format(_("Material table \"{}\" has been reloaded."), materialTableFile));
                    } else {
                        failedToLoad = true;
                    }
                }
            }
        }
    }

    if(failedToLoad){
        MessageView::instance()->putln(
            format(_("Reloading material table \"{}\" failed."), materialTableFile), MessageView::WARNING);
    }

    return materialTable;
}


Item* WorldItem::doDuplicate() const
{
    return new WorldItem(*this);
}


void WorldItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Collision detection"), isCollisionDetectionEnabled(),
                [&](bool on){ enableCollisionDetection(on); return true; });
    putProperty(_("Collision detector"), impl->collisionDetectorType,
                [&](int index){ return impl->selectCollisionDetector(index); });

    FilePathProperty materialFileProperty(impl->materialTableFile, { _("Contact material definition file (*.yaml)") });
    putProperty(_("Material table"), materialFileProperty,
                [&](const string& filename){ setMaterialTableFile(filename); return true; });
}


bool WorldItem::store(Archive& archive)
{
    archive.write("collisionDetection", isCollisionDetectionEnabled());
    archive.write("collisionDetector", impl->collisionDetectorType.selectedSymbol());
    archive.writeRelocatablePath("materialTableFile", impl->materialTableFile);
    return true;
}


bool WorldItem::restore(const Archive& archive)
{
    string symbol;
    if(archive.read("collisionDetector", symbol)){
        selectCollisionDetector(symbol);
    }
    if(archive.get("collisionDetection", false)){
        archive.addPostProcess([&](){ impl->enableCollisionDetection(true); });
    }
    if(archive.readRelocatablePath("materialTableFile", symbol)){
        setMaterialTableFile(symbol);
    }
    return true;
}
