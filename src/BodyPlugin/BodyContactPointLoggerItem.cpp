#include "BodyContactPointLoggerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ControllerLogItem>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/SceneDrawables>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class BodyContactPointLogItem : public ControllerLogItem
{
public:
    BodyContactPointLogItem();
    BodyContactPointLogItem(const BodyContactPointLogItem& org);
    virtual Item* doDuplicate() const override;
};

typedef ref_ptr<BodyContactPointLogItem> BodyContactPointLogItemPtr;


class BodyContactPointLog : public Referenced
{
public:
    vector<vector<Link::ContactPoint>> bodyContactPoints;
};


class BodyContactPointLogEngine : public TimeSyncItemEngine
{
public:
    static BodyContactPointLogEngine* create(
        BodyContactPointLogItem* logItem, BodyContactPointLogEngine* engine0);
    
    BodyContactPointLoggerItemPtr loggerItem;
    BodyContactPointLogItemPtr logItem;

    BodyContactPointLogEngine(BodyContactPointLogItem* logItem, BodyContactPointLoggerItem* loggerItem);
    ~BodyContactPointLogEngine();
    
    virtual bool onTimeChanged(double time) override;
};

}

namespace cnoid {

class BodyContactPointLoggerItem::Impl
{
public:
    ControllerIO* io;
    Body* body;
    SgLineSetPtr contactLineSet;
    SgUpdate update;

    static Impl* getImpl(BodyContactPointLoggerItem* item){ return item->impl; };
    
    void updateScene(const vector<vector<Link::ContactPoint>>& bodyContactPoints);
    void clearScene();
};

}


void BodyContactPointLoggerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodyContactPointLoggerItem, ControllerItem>(N_("BodyContactPointLoggerItem"))
        .addCreationPanel<BodyContactPointLoggerItem>()
        .registerClass<BodyContactPointLogItem, ControllerLogItem>(N_("BodyContactPointLogItem"));

    TimeSyncItemEngineManager::instance()
        ->registerFactory<BodyContactPointLogItem, BodyContactPointLogEngine>(
            BodyContactPointLogEngine::create);
}


BodyContactPointLoggerItem::BodyContactPointLoggerItem()
{
    impl = new Impl;
}


BodyContactPointLoggerItem::BodyContactPointLoggerItem(const BodyContactPointLoggerItem& org)
    : ControllerItem(org)
{
    impl = new Impl;
}


Item* BodyContactPointLoggerItem::doDuplicate() const
{
    return new BodyContactPointLoggerItem(*this);
}


bool BodyContactPointLoggerItem::initialize(ControllerIO* io)
{
    impl->io = io;
    impl->body = io->body();
    
    for(auto& link : impl->body->links()){
        link->mergeSensingMode(Link::LinkContactState);
    }
    
    return io->enableLog();
}


ControllerLogItem* BodyContactPointLoggerItem::createLogItem()
{
    return new BodyContactPointLogItem;
}


void BodyContactPointLoggerItem::log()
{
    auto log = new BodyContactPointLog;
    for(auto& link : impl->body->links()){
        auto& contacts = link->contactPoints();
        log->bodyContactPoints.push_back(link->contactPoints());
    }
    impl->io->outputLog(log);
}


SgNode* BodyContactPointLoggerItem::getScene()
{
    if(!impl->contactLineSet){
        impl->contactLineSet = new SgLineSet;
    }
    return impl->contactLineSet;
}


void BodyContactPointLoggerItem::Impl::updateScene
(const vector<vector<Link::ContactPoint>>& bodyContactPoints)
{
    if(!contactLineSet){
        return;
    }

    contactLineSet->clearLines();
    auto vertices = contactLineSet->getOrCreateVertices();
    vertices->clear();
    int vertexIndex = 0;
    for(auto& points : bodyContactPoints){
        for(auto& point : points){
            auto p = point.position().cast<Vector3f::Scalar>();
            auto f = 1.0e-3 * point.force().cast<Vector3f::Scalar>();
            vertices->push_back(p);
            vertices->push_back(p + f);
            contactLineSet->addLine(vertexIndex, vertexIndex + 1);
            vertexIndex += 2;
        }
    }
    contactLineSet->notifyUpdate(update);
}


void BodyContactPointLoggerItem::Impl::clearScene()
{
    if(contactLineSet){
        contactLineSet->clearLines();
        contactLineSet->notifyUpdate(update);
    }
}


namespace {

BodyContactPointLogItem::BodyContactPointLogItem()
{

}


BodyContactPointLogItem::BodyContactPointLogItem(const BodyContactPointLogItem& org)
    : ControllerLogItem(org)
{

}


Item* BodyContactPointLogItem::doDuplicate() const
{
    return new BodyContactPointLogItem(*this);
}


BodyContactPointLogEngine* BodyContactPointLogEngine::create
(BodyContactPointLogItem* logItem, BodyContactPointLogEngine* engine0)
{
    if(auto loggerItem = logItem->findOwnerItem<BodyContactPointLoggerItem>()){
        if(engine0 && engine0->loggerItem == loggerItem){
            return engine0;
        } else {
            return new BodyContactPointLogEngine(logItem, loggerItem);
        }
    }
    return nullptr;
}


BodyContactPointLogEngine::BodyContactPointLogEngine
(BodyContactPointLogItem* logItem, BodyContactPointLoggerItem* loggerItem)
    : TimeSyncItemEngine(logItem),
      loggerItem(loggerItem),
      logItem(logItem)
{
    loggerItem->sigCheckToggled().connect(
        [&](bool on){
            if(on){
                refresh();
            }
        });
}


BodyContactPointLogEngine::~BodyContactPointLogEngine()
{
    BodyContactPointLoggerItem::Impl::getImpl(loggerItem)->clearScene();
}


bool BodyContactPointLogEngine::onTimeChanged(double time)
{
    if(!loggerItem->isChecked()){
        return false;
    }
    auto log = logItem->log();
    if(log->empty()){
        return false;
    }
    bool isValid;
    int frame = log->frameOfTime(time);
    if(frame < log->numFrames()){
        isValid = true;
    } else {
        isValid = false;
        frame = log->numFrames() - 1;
    }

    if(auto contactPointLog = dynamic_pointer_cast<BodyContactPointLog>(log->at(frame))){
        auto loggerItemImpl = BodyContactPointLoggerItem::Impl::getImpl(loggerItem);
        loggerItemImpl->updateScene(contactPointLog->bodyContactPoints);
    }
    
    return isValid;
}

}
