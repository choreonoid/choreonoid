#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/ControllerItem>
#include <cnoid/ControllerLogItem>
#include <cnoid/TimeSyncItemEngine>
#include <cnoid/RenderableItem>
#include <cnoid/SceneDrawables>

using namespace std;
using namespace cnoid;

namespace {

class ContactPointLoggerItem : public ControllerItem, public RenderableItem
{
public:
    ControllerIO* io;
    Body* body;
    SgLineSetPtr contactLineSet;
    SgUpdate update;
    
    ContactPointLoggerItem();
    ContactPointLoggerItem(const ContactPointLoggerItem& org);

    virtual bool initialize(ControllerIO* io) override;
    virtual ControllerLogItem* createLogItem() override;
    virtual void log() override;

    virtual SgNode* getScene() override;
    void updateScene(const vector<vector<Link::ContactPoint>>& bodyContactPoints);
    void clearScene();

protected:
    virtual Item* doDuplicate() const;
};

typedef ref_ptr<ContactPointLoggerItem> ContactPointLoggerItemPtr;


class ContactPointLogItem : public ControllerLogItem
{
public:
    ContactPointLogItem();
    ContactPointLogItem(const ContactPointLogItem& org);
    virtual Item* doDuplicate() const override;
};

typedef ref_ptr<ContactPointLogItem> ContactPointLogItemPtr;


class ContactPointLog : public Referenced
{
public:
    vector<vector<Link::ContactPoint>> bodyContactPoints;
};


class ContactPointLogEngine : public TimeSyncItemEngine
{
public:
    ContactPointLoggerItemPtr loggerItem;
    ContactPointLogItemPtr logItem;

    ContactPointLogEngine(ContactPointLoggerItem* loggerItem, ContactPointLogItem* logItem);
    ~ContactPointLogEngine();
    
    virtual bool onTimeChanged(double time) override;
};


class ContactPointLoggerPlugin : public Plugin
{
public:
    ContactPointLoggerPlugin();
    virtual bool initialize() override;
};

}


ContactPointLoggerItem::ContactPointLoggerItem()
{

}


ContactPointLoggerItem::ContactPointLoggerItem(const ContactPointLoggerItem& org)
    : ControllerItem(org)
{

}

Item* ContactPointLoggerItem::doDuplicate() const
{
    return new ContactPointLoggerItem(*this);
}


bool ContactPointLoggerItem::initialize(ControllerIO* io)
{
    this->io = io;
    body = io->body();
    
    for(auto& link : body->links()){
        link->mergeSensingMode(Link::LinkContactState);
    }
    
    return io->enableLog();
}


ControllerLogItem* ContactPointLoggerItem::createLogItem()
{
    return new ContactPointLogItem;
}


void ContactPointLoggerItem::log()
{
    auto log = new ContactPointLog;
    for(auto& link : body->links()){
        auto& contacts = link->contactPoints();
        log->bodyContactPoints.push_back(link->contactPoints());
    }
    io->outputLog(log);
}


SgNode* ContactPointLoggerItem::getScene()
{
    if(!contactLineSet){
        contactLineSet = new SgLineSet;
    }
    return contactLineSet;
}


void ContactPointLoggerItem::updateScene(const vector<vector<Link::ContactPoint>>& bodyContactPoints)
{
    getScene();

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


void ContactPointLoggerItem::clearScene()
{
    if(contactLineSet){
        contactLineSet->clearLines();
        contactLineSet->notifyUpdate(update);
    }
}


ContactPointLogItem::ContactPointLogItem()
{

}


ContactPointLogItem::ContactPointLogItem(const ContactPointLogItem& org)
    : ControllerLogItem(org)
{

}


Item* ContactPointLogItem::doDuplicate() const
{
    return new ContactPointLogItem(*this);
}


ContactPointLogEngine::ContactPointLogEngine(ContactPointLoggerItem* loggerItem, ContactPointLogItem* logItem)
    : TimeSyncItemEngine(logItem),
      loggerItem(loggerItem),
      logItem(logItem)
{

}


ContactPointLogEngine::~ContactPointLogEngine()
{
    loggerItem->clearScene();
}


bool ContactPointLogEngine::onTimeChanged(double time)
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

    if(auto contactPointLog = dynamic_pointer_cast<ContactPointLog>(log->at(frame))){
        loggerItem->updateScene(contactPointLog->bodyContactPoints);
    }
    
    return isValid;
}


static TimeSyncItemEngine* createContactPointLogEngine(ContactPointLogItem* logItem)
{
    if(auto loggerItem = logItem->findOwnerItem<ContactPointLoggerItem>()){
        return new ContactPointLogEngine(loggerItem, logItem);
    }
    return nullptr;
}


ContactPointLoggerPlugin::ContactPointLoggerPlugin()
    : Plugin("ContactPointLoggerPlugin")
{
    require("Body");
}


bool ContactPointLoggerPlugin::initialize()
{
    itemManager()
        .registerClass<ContactPointLoggerItem, ControllerItem>("ContactPointLoggerItem")
        .addCreationPanel<ContactPointLoggerItem>()
        .registerClass<ContactPointLogItem, ControllerLogItem>("ContactPointLogItem");

    TimeSyncItemEngineManager::instance()
        ->registerFactory<ContactPointLogItem>(createContactPointLogEngine);

    return true;
}


CNOID_IMPLEMENT_PLUGIN_ENTRY(ContactPointLoggerPlugin);
