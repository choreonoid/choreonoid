/**
   @author Shin'ichiro Nakaoka
*/

#include "OnlineViewerServer.h"
#include <cnoid/MessageView>
#include <cnoid/ItemTreeView>
#include <cnoid/TimeBar>
#include <cnoid/RootItem>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/WorldItem>
#include <cnoid/CorbaUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/LazyCaller>
#include <fmt/format.h>
#include <QRegExp>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace OpenHRP;
using fmt::format;

namespace cnoid {

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
    
class OnlineViewerServerImpl
{
public:
    typedef std::map<std::string, BodyItemInfo> BodyItemInfoMap;
    BodyItemInfoMap bodyItemInfoMap;
    TimeBar* timeBar;
    MessageView* mv;

    OnlineViewerServerImpl();
    ~OnlineViewerServerImpl();
    BodyItemInfo* findInfo(const string& name);
    void load(string name, string url);
    void registerBodyItem(BodyItemPtr bodyItem);
    void onBodyItemNameChanged(BodyItem* bodyItem, const std::string& oldName);
    void onBodyItemDetachedFromRoot(BodyItem* bodyItem);
    void forEachBody(
        const WorldState& state,
        std::function<void(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)> callback);
    void drawScene(const OpenHRP::WorldState& state);
    void updateBodyState(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time);
    void update(const OpenHRP::WorldState& state);
    void updateLog(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time);
    void resetLogItem(BodyItemInfo* info, BodyMotionItem* newLogItem);
    void clearLog();
    void clearData();
    void setLineWidth(::CORBA::Float width);
    void setLineScale(::CORBA::Float scale);
    ::CORBA::Boolean getPosture(const char* robotId, OpenHRP::DblSequence_out posture);

#ifdef OPENHRP_3_1
    void setLogName(string name);
#endif
};
}


OnlineViewerServer::OnlineViewerServer()
{
    impl = new OnlineViewerServerImpl();
}


OnlineViewerServerImpl::OnlineViewerServerImpl()
{
    timeBar = TimeBar::instance();
    mv = MessageView::instance();
}


OnlineViewerServer::~OnlineViewerServer()
{
    delete impl;

    PortableServer::POA_var poa = _default_POA();
    PortableServer::ObjectId_var id = poa->servant_to_id(this);
    poa->deactivate_object(id);
}


OnlineViewerServerImpl::~OnlineViewerServerImpl()
{
    
}


BodyItemInfo* OnlineViewerServerImpl::findInfo(const string& name)
{
    BodyItemInfoMap::iterator p = bodyItemInfoMap.find(name);
    if(p != bodyItemInfoMap.end()){
        return &p->second;
    }
    return 0;
}    


void OnlineViewerServer::load(const char* name, const char* url)
{
    // Wait for the load function to finish because
    // the function does MessageView::flush(), which may execute other OnlineViewer's functions
    // before finishing the loading.
    callSynchronously(std::bind(&OnlineViewerServerImpl::load, impl, string(name), string(url)));
}


void OnlineViewerServerImpl::load(string name, string url)
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
            registerBodyItem(bodyItem);
            return;
        }
    }
    // load a new body item
    BodyItemPtr bodyItem = new BodyItem();
    mv->putln(format(_("OnlineViewer: Loading \"{0}\" at \"{1}\"."), name, url));
    mv->flush();
    if(!bodyItem->load(filepath)){
        mv->putln(format(_("OnlineViewer: Loading \"{0}\" failed."), name));

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


void OnlineViewerServerImpl::registerBodyItem(BodyItemPtr bodyItem)
{
    BodyItemInfo info;

    info.bodyItem = bodyItem;
    info.needToSelectLogItem = true;

    info.bodyItemConnections.add(
        bodyItem->sigNameChanged().connect(
            std::bind(&OnlineViewerServerImpl::onBodyItemNameChanged, this, bodyItem.get(), _1)));
    info.bodyItemConnections.add(
        bodyItem->sigDisconnectedFromRoot().connect(
            std::bind(&OnlineViewerServerImpl::onBodyItemDetachedFromRoot, this, bodyItem.get())));

    bodyItemInfoMap.insert(make_pair(bodyItem->name(), info));
}


void OnlineViewerServerImpl::onBodyItemNameChanged(BodyItem* bodyItem, const std::string& oldName)
{
    bodyItemInfoMap.erase(oldName);
    mv->putln(format(_("OnlineViewer: \"{}\" is unregistered because the name has been changed."),
                     oldName));
}


void OnlineViewerServerImpl::onBodyItemDetachedFromRoot(BodyItem* bodyItem)
{
    bodyItemInfoMap.erase(bodyItem->name());
    mv->putln(format(_("OnlineViewer: \"{}\" has been removed."), bodyItem->name()));
}


void OnlineViewerServerImpl::forEachBody
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


void OnlineViewerServer::drawScene(const WorldState& state)
{
    callLater(std::bind(&OnlineViewerServerImpl::drawScene, impl, state));
}


void OnlineViewerServerImpl::drawScene(const WorldState& state)
{
    forEachBody(state, std::bind(&OnlineViewerServerImpl::updateBodyState, this, _1, _2, _3, _4));
}


void OnlineViewerServerImpl::updateBodyState
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


void OnlineViewerServer::update(const WorldState& state)
{
    callLater(std::bind(&OnlineViewerServerImpl::update, impl, state));
}


void OnlineViewerServerImpl::update(const WorldState& state)
{
    forEachBody(state, std::bind(&OnlineViewerServerImpl::updateLog, this, _1, _2, _3, _4));
}


void OnlineViewerServerImpl::updateLog
(BodyItemInfo* info, const LinkPositionSequence& links, int numLinks, double time)
{
    BodyMotionItem* motionItem = info->logItem;
    if(!motionItem){
        motionItem = info->bodyItem->findChildItem<BodyMotionItem>(info->logName);
        if(!motionItem){
            motionItem = new BodyMotionItem();
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


void OnlineViewerServerImpl::resetLogItem(BodyItemInfo* info, BodyMotionItem* newLogItem)
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
                std::bind(&OnlineViewerServerImpl::resetLogItem, this, info, (BodyMotionItem*)0)));
        info->logItemConnections.add(
            newLogItem->sigNameChanged().connect(
                std::bind(&OnlineViewerServerImpl::resetLogItem, this, info, (BodyMotionItem*)0)));
    }
}


#ifdef OPENHRP_3_1
void OnlineViewerServer::setLogName(const char* name)
{
    callLater(std::bind(&OnlineViewerServerImpl::setLogName, impl, string(name)));
}

void OnlineViewerServerImpl::setLogName(string name)
{
    BodyItemInfo* info = findInfo(name);
    if(info){
        info->logName = name;
        if(info->logItem && info->logItem->name() != name){
            info->logItem = 0;
        }
    }
}
#endif


void OnlineViewerServer::clearLog()
{
    callLater(std::bind(&OnlineViewerServerImpl::clearLog, impl));
}


void OnlineViewerServerImpl::clearLog()
{
    for(BodyItemInfoMap::iterator p = bodyItemInfoMap.begin(); p != bodyItemInfoMap.end(); ++p){
        BodyItemInfo& info = p->second;
        if(info.logItem){
            info.logItem->motion()->setNumFrames(0);
        }
    }
}


void OnlineViewerServer::clearData()
{
    callLater(std::bind(&OnlineViewerServerImpl::clearData, impl));
}


void OnlineViewerServerImpl::clearData()
{
    bodyItemInfoMap.clear();
}


void OnlineViewerServer::setLineWidth(::CORBA::Float width)
{

}


void OnlineViewerServer::setLineScale(::CORBA::Float scale)
{

}


::CORBA::Boolean OnlineViewerServer::getPosture(const char* robotId, DblSequence_out posture)
{
    return false;
}
