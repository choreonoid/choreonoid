/**
   @author Shizuko Hattori
*/

#include "OpenHRPOnlineViewerItem.h"

#ifdef OPENHRP_3_0
#include <cnoid/corba/OpenHRP/3.0/OnlineViewer.hh>
#elif OPENHRP_3_1
#include <cnoid/corba/OpenHRP/3.1/OnlineViewer.hh>
#endif

#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <cnoid/CorbaUtil>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/ConnectionSet>
#include <cnoid/TimeBar>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/WorldItem>
#include <cnoid/LazyCaller>
#include <cnoid/SceneCollision>
#include <cnoid/CollisionSeqItem>
#include <cnoid/CollisionSeq>
#include <fmt/format.h>
#include <QRegExp>
#include <sstream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace OpenHRP;
using fmt::format;

namespace cnoid {

#ifdef OPENHRP_3_0
#define ITEM_NAME N_("OpenHRP3.0OnlineViewerItem")
#elif OPENHRP_3_1
#define ITEM_NAME N_("OpenHRP3.1OnlineViewerItem")
#endif


struct BodyItemInfo {
    BodyItemPtr bodyItem;
    std::string logName;
    BodyMotionItemPtr logItem;
    bool needToSelectLogItem;
    ConnectionSet bodyItemConnections;
    ConnectionSet logItemConnections;
    BodyItemInfo()
        : logName("OnlineViewerLog") {
        needToSelectLogItem = false;
    }
    ~BodyItemInfo() {
        bodyItemConnections.disconnect();
        logItemConnections.disconnect();
    }
};

class OpenHRPOnlineViewerItemImpl
        : virtual public POA_OpenHRP::OnlineViewer,
          virtual public PortableServer::RefCountServantBase
{
public:
    OpenHRPOnlineViewerItem* self;
    ostream& os;

    OpenHRPOnlineViewerItemImpl(OpenHRPOnlineViewerItem* self);
    OpenHRPOnlineViewerItemImpl(OpenHRPOnlineViewerItem* self,
            const OpenHRPOnlineViewerItemImpl& org);
    ~OpenHRPOnlineViewerItemImpl();
    void setServerName(const string& newName);
    void init();

    virtual void load(const char* name, const char* url);
    virtual void update(const OpenHRP::WorldState& state);
    virtual void clearLog();
    virtual void clearData();
    virtual void drawScene(const OpenHRP::WorldState& state);
    virtual void setLineWidth(::CORBA::Float width);
    virtual void setLineScale(::CORBA::Float scale);
    virtual ::CORBA::Boolean getPosture(const char* robotId, OpenHRP::DblSequence_out posture);

#ifdef OPENHRP_3_1
    virtual void setLogName(const char* name);
#endif

    string serverName;
    typedef std::map<std::string, BodyItemInfo> BodyItemInfoMap;
    BodyItemInfoMap bodyItemInfoMap;
    MessageView* mv;
    TimeBar* timeBar;
    CollisionLinkPairListPtr collisions;
    SceneCollisionPtr sceneCollision;
    string collisionLogName;
    WorldItemPtr worldItem;
    CollisionSeqItem* collisionSeqItem;
    bool needToSelectCollisionLogItem;
    ConnectionSet worldItemConnections;
    ConnectionSet collisionSeqItemConnections;

    BodyItemInfo* findInfo(const string& name);
    void loadsub(string name, string url);
    void registerBodyItem(BodyItemPtr bodyItem);
    void onBodyItemNameChanged(BodyItem* bodyItem, const std::string& oldName);
    void onBodyItemDetachedFromRoot(BodyItem* bodyItem);
    void forEachBody(
        const WorldState& state,
        std::function<void(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)> callback);
    void drawScenesub(const OpenHRP::WorldState& state);
    void updateBodyState(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time);
    void updatesub(const OpenHRP::WorldState& state);
    void updateLog(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time);
    void resetLogItem(BodyItemInfo* info, BodyMotionItem* newLogItem);
    void resetCollisionLogItem(CollisionSeqItem* newCollisionLogItem);
    void clearLogsub();
    void clearDatasub();
    //void setLineWidth(::CORBA::Float width);
    //void setLineScale(::CORBA::Float scale);
    //::CORBA::Boolean getPosture(const char* robotId, OpenHRP::DblSequence_out posture);
    void updateCollision(const WorldState& state, CollisionLinkPairList* collisions);
    void onWorldItemDetachedFromRoot() { worldItem = 0; }

#ifdef OPENHRP_3_1
    void setLogNamesub(string name);
#endif

};
}


void OpenHRPOnlineViewerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<OpenHRPOnlineViewerItem>(ITEM_NAME)
        .addCreationPanel<OpenHRPOnlineViewerItem>();
}


OpenHRPOnlineViewerItem::OpenHRPOnlineViewerItem()
{
    impl = new OpenHRPOnlineViewerItemImpl(this);
}


OpenHRPOnlineViewerItemImpl::OpenHRPOnlineViewerItemImpl(OpenHRPOnlineViewerItem* self)
    : self(self),
      os(MessageView::instance()->cout()),
      serverName("OnlineViewer")
{
    init();
}


OpenHRPOnlineViewerItem::OpenHRPOnlineViewerItem(const OpenHRPOnlineViewerItem& org)
    : Item(org)
{
    impl = new  OpenHRPOnlineViewerItemImpl(this, *org.impl);
}


OpenHRPOnlineViewerItemImpl::OpenHRPOnlineViewerItemImpl(OpenHRPOnlineViewerItem* self,
        const OpenHRPOnlineViewerItemImpl& org)
    : self(self),
      os(MessageView::instance()->cout())
{
    serverName = org.serverName;
    init();
}


void OpenHRPOnlineViewerItemImpl::init()
{
    timeBar = TimeBar::instance();
    mv = MessageView::instance();
    collisions = std::make_shared<CollisionLinkPairList>();
    sceneCollision = new SceneCollision(collisions);
    sceneCollision->setName("Collisions");
    collisionLogName = "OnlineViewerLog";
    worldItem = 0;
    collisionSeqItem = 0;
    needToSelectCollisionLogItem = true;
}


OpenHRPOnlineViewerItem::~OpenHRPOnlineViewerItem()
{
    delete impl;
}


OpenHRPOnlineViewerItemImpl::~OpenHRPOnlineViewerItemImpl()
{
    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);

    worldItemConnections.disconnect();
}


Item* OpenHRPOnlineViewerItem::doDuplicate() const
{
    return new OpenHRPOnlineViewerItem(*this);
}


void OpenHRPOnlineViewerItem::onConnectedToRoot()
{
    NamingContextHelper* ncHelper = getDefaultNamingContextHelper();
    string orgName = impl->serverName;
    for(int i=0; ; i++){
        OnlineViewer_var olvServer = ncHelper->findObject<OnlineViewer>(impl->serverName);
        if(!CORBA::is_nil(olvServer))
            if(ncHelper->isObjectAlive(olvServer)){
                impl->os << impl->serverName << " already exists.  I will change the server name." << endl;
                std::stringstream ss;
                ss << orgName << i;
                impl->serverName = ss.str();
            }else{
                ncHelper->unbind(impl->serverName);
                break;
            }
        else
            break;
    }

    ncHelper->bindObject(impl->_this(), impl->serverName);
}


void OpenHRPOnlineViewerItem::onDisconnectedFromRoot()
{
    getDefaultNamingContextHelper()->unbind(impl->serverName);
}


void OpenHRPOnlineViewerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Server name"), impl->serverName,
            std::bind(&OpenHRPOnlineViewerItemImpl::setServerName, impl, _1), true);
}


bool OpenHRPOnlineViewerItem::store(Archive& archive)
{
    archive.write("serverName", impl->serverName);
    return true;
}


bool OpenHRPOnlineViewerItem::restore(const Archive& archive)
{
    impl->setServerName(archive.get("serverName", impl->serverName));
    return true;
}


void OpenHRPOnlineViewerItemImpl::setServerName(const string& newName)
{
    if(serverName != newName){
        NamingContextHelper* ncHelper = getDefaultNamingContextHelper();
        ncHelper->unbind(serverName);
        serverName = newName;
        ncHelper->bindObject(_this(), serverName);
    }
}


BodyItemInfo* OpenHRPOnlineViewerItemImpl::findInfo(const string& name)
{
    BodyItemInfoMap::iterator p = bodyItemInfoMap.find(name);
    if(p != bodyItemInfoMap.end()){
        return &p->second;
    }
    return 0;
}    


void OpenHRPOnlineViewerItemImpl::load(const char* name, const char* url)
{
    // Wait for the load function to finish because
    // the function does MessageView::flush(), which may execute other OnlineViewer's functions
    // before finishing the loading.
    callSynchronously(std::bind(&OpenHRPOnlineViewerItemImpl::loadsub, this, string(name), string(url)));
}


void OpenHRPOnlineViewerItemImpl::loadsub(string name, string url)
{
    string filepath;

    QRegExp filePattern("(\\w+)://(.+)");
    if(filePattern.exactMatch(url.c_str())){
        string protocol = filePattern.cap(1).toStdString();
        if(protocol == "file"){
            filepath = filePattern.cap(2).toStdString();
        } else {
            mv->putln(
                format(_("OnlineViewer: The model file at \"{0}\" cannot be read. {1} protocol is not supported."),
                       url, protocol));
            return;
        }
    } else {
        filepath = url;
    }
        
    // search for registered body items
    BodyItemInfo* info = findInfo(name);
    if(info && info->bodyItem->filePath() == filepath){
        info->needToSelectLogItem = true;
        // mv->putln(fmt(_("OnlineViewer: \"%1%\" at \"%2%\" has already been loaded.")) % name % url);
        return;
    }
    
    // search for existing body items
    RootItem* rootItem = RootItem::instance();
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(rootItem);
    for(size_t i=0; i < bodyItems.size(); ++i){
        BodyItemPtr bodyItem = bodyItems[i];
        if(bodyItem->name() == name && bodyItem->filePath() == filepath){
            if(!bodyItem->load(filepath)){
                mv->putln(format(_("OnlineViewer: Loading \"{}\" failed."), name));
                return;
            }
            registerBodyItem(bodyItem);
            return;
        }
    }
    // load a new body item
    BodyItemPtr bodyItem = new BodyItem();
    mv->putln(format(_("OnlineViewer: Loading \"{0}\" at \"{1}\"."),
                     name, url));
    mv->flush();
    if(!bodyItem->load(filepath)){
        mv->putln(format(_("OnlineViewer: Loading \"{}\" failed."), name));

    } else {
        bodyItem->setName(name);
        ItemList<WorldItem> worldItems;
        if(worldItems.extractChildItems(rootItem)){
            worldItems.front()->addChildItem(bodyItem);
        } else {
            rootItem->addChildItem(bodyItem);
        }
        ItemTreeView::instance()->checkItem(bodyItem, true);
        registerBodyItem(bodyItem);
    }
}


void OpenHRPOnlineViewerItemImpl::registerBodyItem(BodyItemPtr bodyItem)
{
    BodyItemInfo info;

    info.bodyItem = bodyItem;
    info.needToSelectLogItem = true;

    info.bodyItemConnections.add(
        bodyItem->sigNameChanged().connect(
            std::bind(&OpenHRPOnlineViewerItemImpl::onBodyItemNameChanged, this, bodyItem.get(), _1)));
    info.bodyItemConnections.add(
        bodyItem->sigDisconnectedFromRoot().connect(
            std::bind(&OpenHRPOnlineViewerItemImpl::onBodyItemDetachedFromRoot, this, bodyItem.get())));

    bodyItemInfoMap.insert(make_pair(bodyItem->name(), info));
}


void OpenHRPOnlineViewerItemImpl::onBodyItemNameChanged(BodyItem* bodyItem, const std::string& oldName)
{
    bodyItemInfoMap.erase(oldName);
    mv->putln(format(_("OnlineViewer: \"{}\" is unregistered because the name has been changed."),
                     oldName));
}


void OpenHRPOnlineViewerItemImpl::onBodyItemDetachedFromRoot(BodyItem* bodyItem)
{
    bodyItemInfoMap.erase(bodyItem->name());
    mv->putln(format(_("OnlineViewer: \"{}\" has been removed."), bodyItem->name()));
}


void OpenHRPOnlineViewerItemImpl::forEachBody
(const WorldState& state,
 std::function<void(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)> callback)
{
    int numBodies = state.characterPositions.length();
    for(int i=0; i < numBodies; ++i){
        const CharacterPosition& bodyPosition = state.characterPositions[i];
        BodyItemInfo* info = findInfo(bodyPosition.characterName.in());
        if(info){
            const LinkPositionSequence& links = bodyPosition.linkPositions;
            const BodyPtr& body = info->bodyItem->body();
            int numLinks = links.length();
            if(body->numLinks() < numLinks){
                numLinks = body->numLinks();
            }
            callback(info, links, numLinks, state.time);
        }
    }
    timeBar->setTime(state.time);
}


void OpenHRPOnlineViewerItemImpl::drawScene(const WorldState& state)
{
    callLater(std::bind(&OpenHRPOnlineViewerItemImpl::drawScenesub, this, state));
}


void OpenHRPOnlineViewerItemImpl::drawScenesub(const WorldState& state)
{
    forEachBody(state, std::bind(&OpenHRPOnlineViewerItemImpl::updateBodyState, this, _1, _2, _3, _4));

    updateCollision(state, collisions.get());
    sceneCollision->setDirty();
}


void OpenHRPOnlineViewerItemImpl::updateCollision(const WorldState& state, CollisionLinkPairList* collisions)
{
    collisions->clear();
    int n = state.collisions.length();
    for(int i=0; i < n; i++){
        const OpenHRP::Collision& source = state.collisions[i];
        CollisionLinkPairPtr dest = std::make_shared<CollisionLinkPair>();
        int numPoints = source.points.length();
        for(int j=0; j < numPoints; j++){
           // std::cout << source.points[j].position[0] << " " << source.points[j].position[1] << " " << source.points[j].position[2] << std::endl;
           // std::cout << source.points[j].normal[0] << " " << source.points[j].normal[1] << " " << source.points[j].normal[2] << std::endl;
            const CollisionPoint& point = source.points[j];
            dest->collisions.push_back(Collision());
            Collision& col = dest->collisions.back();
            col.point = Vector3(point.position[0], point.position[1], point.position[2]);
            col.normal = Vector3(point.normal[0], point.normal[1], point.normal[2]);
            col.depth = point.idepth;
        }
        //std::cout <<std::endl;
        for(int j=0; j < 2; j++){
            BodyItemInfo* info = findInfo(i? source.pair.charName2.in() : source.pair.charName1.in());
            if(info){
                const BodyPtr& body = info->bodyItem->body();
                dest->body[j] = body;
                dest->link[j] = body->link(i? source.pair.linkName2.in() : source.pair.linkName1.in());
            }
        }
        collisions->push_back(dest);
    }
}


void OpenHRPOnlineViewerItemImpl::updateBodyState
(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)
{
    const BodyPtr& body = info->bodyItem->body();
    for(int j=0; j < numLinks; ++j){
        Link* link = body->link(j);
        link->p() = Eigen::Map<Vector3>(const_cast<double*>(links[j].p));
        link->setAttitude(Eigen::Map<Matrix3>(const_cast<double*>(links[j].R)).transpose());
    }
    info->bodyItem->notifyKinematicStateChange();
}


void OpenHRPOnlineViewerItemImpl::update(const WorldState& state)
{
    callLater(std::bind(&OpenHRPOnlineViewerItemImpl::updatesub, this, state));
}


void OpenHRPOnlineViewerItemImpl::updatesub(const WorldState& state)
{
    if(!worldItem){
        RootItem* rootItem = RootItem::instance();
        ItemList<WorldItem> worldItems;
        if(worldItems.extractChildItems(rootItem)){
            worldItem = worldItems.front();
        } else {
            worldItem = new WorldItem();
            worldItem->setName("World");
            rootItem->addChildItem(worldItem);
            ItemTreeView::instance()->checkItem(worldItem, true);
        }
       worldItemConnections.add(
               worldItem->sigDisconnectedFromRoot().connect(
                    std::bind(&OpenHRPOnlineViewerItemImpl::onWorldItemDetachedFromRoot, this)));
    }

    if(!collisionSeqItem){
        collisionSeqItem = worldItem->findChildItem<CollisionSeqItem>(collisionLogName);
        if(!collisionSeqItem){
            collisionSeqItem = new CollisionSeqItem();
            collisionSeqItem->setTemporal();
            collisionSeqItem->setName(collisionLogName);
            worldItem->addChildItem(collisionSeqItem);
        }
        resetCollisionLogItem(collisionSeqItem);
        needToSelectCollisionLogItem = true;
    }

    if(needToSelectCollisionLogItem){
        ItemTreeView::instance()->selectItem(collisionSeqItem, true);
        needToSelectCollisionLogItem = false;
    }

    const CollisionSeqPtr& colSeq = collisionSeqItem->collisionSeq();
    int frame = colSeq->frameOfTime(state.time);
    int lastFrame = std::max(0, std::min(frame, colSeq->numFrames()));
    colSeq->setNumFrames(frame + 1);

    CollisionLinkPairListPtr collisionPairs = std::make_shared<CollisionLinkPairList>();
    updateCollision(state, collisionPairs.get());
    for(int i=lastFrame; i <= frame; ++i){
        CollisionSeq::Frame collisionPairs0 = colSeq->frame(i);
        collisionPairs0[0] = collisionPairs;
    }

    forEachBody(state, std::bind(&OpenHRPOnlineViewerItemImpl::updateLog, this, _1, _2, _3, _4));
}


void OpenHRPOnlineViewerItemImpl::updateLog
(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)
{
    BodyMotionItem* motionItem = info->logItem.get();
    if(!motionItem){
        motionItem = info->bodyItem->findChildItem<BodyMotionItem>(info->logName);
        if(!motionItem){
            motionItem = new BodyMotionItem();
            motionItem->setTemporal();
            motionItem->setName(info->logName);
            info->bodyItem->addChildItem(motionItem);
        }
        resetLogItem(info, motionItem);
    }

    if(info->needToSelectLogItem){
        ItemTreeView::instance()->selectItem(motionItem, true);
        info->needToSelectLogItem = false;
    }

    MultiSE3Seq& seq = *motionItem->motion()->linkPosSeq();
    int frame = seq.frameOfTime(time);
    int lastFrame = std::max(0, std::min(frame, seq.numFrames()));
    seq.setNumFrames(frame + 1);

    Body* body = info->bodyItem->body();
    for(int i=lastFrame; i <= frame; ++i){
        MultiSE3Seq::Frame positions = seq.frame(i);
        for(int j=0; j < numLinks; ++j){
            SE3& se3 = positions[j];
            se3.translation() = Eigen::Map<Vector3>(const_cast<double*>(links[j].p));
            Matrix3 Rs = body->link(j)->Rs().transpose();
            se3.rotation() = Eigen::Map<Matrix3>(const_cast<double*>(links[j].R)).transpose() * Rs;
        }
    }
}


void OpenHRPOnlineViewerItemImpl::resetLogItem(BodyItemInfo* info, BodyMotionItem* newLogItem)
{
    info->logItemConnections.disconnect();
    
    info->logItem = newLogItem;

    if(newLogItem){
        BodyMotionPtr motion = newLogItem->motion();
        motion->jointPosSeq()->setDimension(0, 0);
        motion->linkPosSeq()->setNumParts(info->bodyItem->body()->numLinks());
        motion->setFrameRate(timeBar->frameRate());

        info->logItemConnections.add(
            newLogItem->sigPositionChanged().connect(
                std::bind(&OpenHRPOnlineViewerItemImpl::resetLogItem, this, info, (BodyMotionItem*)0)));
        info->logItemConnections.add(
            newLogItem->sigNameChanged().connect(
                std::bind(&OpenHRPOnlineViewerItemImpl::resetLogItem, this, info, (BodyMotionItem*)0)));
    }
}


void OpenHRPOnlineViewerItemImpl::resetCollisionLogItem(CollisionSeqItem* collisionSeqItem_)
{
    collisionSeqItemConnections.disconnect();

    collisionSeqItem = collisionSeqItem_;

    if(collisionSeqItem){
        CollisionSeqPtr colSeq = collisionSeqItem->collisionSeq();
        colSeq->setFrameRate(timeBar->frameRate());
        colSeq->setNumParts(1);

        collisionSeqItemConnections.add(
            collisionSeqItem->sigPositionChanged().connect(
                std::bind(&OpenHRPOnlineViewerItemImpl::resetCollisionLogItem, this, (CollisionSeqItem*)0)));
        collisionSeqItemConnections.add(
            collisionSeqItem->sigNameChanged().connect(
                std::bind(&OpenHRPOnlineViewerItemImpl::resetCollisionLogItem, this, (CollisionSeqItem*)0)));
    }
}


#ifdef OPENHRP_3_1
void OpenHRPOnlineViewerItemImpl::setLogName(const char* name)
{
    callLater(std::bind(&OpenHRPOnlineViewerItemImpl::setLogNamesub, this, string(name)));
}

void OpenHRPOnlineViewerItemImpl::setLogNamesub(string name)
{
    BodyItemInfo* info = findInfo(name);
    if(info){
        info->logName = name;
        if(info->logItem && info->logItem->name() != name){
            info->logItem = 0;
        }
    }

    collisionLogName = name;
    if(collisionSeqItem && collisionSeqItem->name() != name){
        collisionSeqItem = 0;
    }
}
#endif


void OpenHRPOnlineViewerItemImpl::clearLog()
{
    callLater(std::bind(&OpenHRPOnlineViewerItemImpl::clearLogsub, this));
}


void OpenHRPOnlineViewerItemImpl::clearLogsub()
{
    for(BodyItemInfoMap::iterator p = bodyItemInfoMap.begin(); p != bodyItemInfoMap.end(); ++p){
        BodyItemInfo& info = p->second;
        if(info.logItem){
            info.logItem->motion()->setNumFrames(0);
        }
    }

    if (collisionSeqItem){
        collisionSeqItem->collisionSeq()->setNumFrames(0);
    }
}


void OpenHRPOnlineViewerItemImpl::clearData()
{
    callLater(std::bind(&OpenHRPOnlineViewerItemImpl::clearDatasub, this));
}


void OpenHRPOnlineViewerItemImpl::clearDatasub()
{
    bodyItemInfoMap.clear();
}


void OpenHRPOnlineViewerItemImpl::setLineWidth(::CORBA::Float width)
{

}


void OpenHRPOnlineViewerItemImpl::setLineScale(::CORBA::Float scale)
{

}


::CORBA::Boolean OpenHRPOnlineViewerItemImpl::getPosture(const char* robotId, DblSequence_out posture)
{
    return false;
}


SgNode* OpenHRPOnlineViewerItem::getScene()
{
    return impl->sceneCollision;
}
