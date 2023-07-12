#include "BodyStateLoggerItem.h"
#include <cnoid/ItemManager>
#include <cnoid/BodyMotion>
#include <cnoid/MultiValueSeq>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <mutex>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyStateLoggerItem::Impl
{
public:
    ControllerIO* io;
    Body* ioBody;
    int numJoints;
    shared_ptr<BodyMotion> logBodyMotion;
    shared_ptr<MultiValueSeq> jointEffortLog;
    shared_ptr<MultiValueSeq> jointEffortBuf;
    ScopedConnection logFlushConnection;
    std::mutex logMutex;
    bool isOnlineLoggingMode;
    bool isOnlineLoggingModeProperty;

    Impl();
    bool initialize(ControllerIO* io);
    bool start();
    void input();
    void flushLog();
};

}


void BodyStateLoggerItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<BodyStateLoggerItem, ControllerItem>(N_("BodyStateLoggerItem"))
        .addCreationPanel<BodyStateLoggerItem>();
}


BodyStateLoggerItem::BodyStateLoggerItem()
{
    impl = new Impl;
    setNoDelayMode(false);
}


BodyStateLoggerItem::BodyStateLoggerItem(const BodyStateLoggerItem& org)
    : ControllerItem(org)
{
    impl = new Impl;
}


BodyStateLoggerItem::Impl::Impl()
{
    isOnlineLoggingModeProperty = true;
}


Item* BodyStateLoggerItem::doCloneItem(CloneMap* /* cloneMap */) const
{
    return new BodyStateLoggerItem(*this);
}


bool BodyStateLoggerItem::initialize(ControllerIO* io)
{
    return impl->initialize(io);
}


bool BodyStateLoggerItem::Impl::initialize(ControllerIO* io)
{
    this->io = io;
    ioBody = io->body();
    numJoints = ioBody->numJoints();
    for(auto& joint : ioBody->joints()){
        joint->mergeSensingMode(Link::JointEffort);
    }
    logBodyMotion.reset();
    jointEffortLog.reset();
    return true;
}


bool BodyStateLoggerItem::start()
{
    return impl->start();
}


bool BodyStateLoggerItem::Impl::start()
{
    logBodyMotion = io->logBodyMotion();
    if(!logBodyMotion){
        return false;
    }
    jointEffortLog = logBodyMotion->getOrCreateExtraSeq<MultiValueSeq>(BodyMotion::jointEffortContentName());
    jointEffortLog->setNumFrames(0);
    jointEffortLog->setOffsetTime(0.0);
    jointEffortLog->setNumParts(numJoints);

    if(!jointEffortBuf){
        jointEffortBuf = make_shared<MultiValueSeq>();
    }
    jointEffortBuf->setNumFrames(0);
    jointEffortBuf->setNumParts(numJoints);

    isOnlineLoggingMode = isOnlineLoggingModeProperty;
    if(isOnlineLoggingMode){
        logFlushConnection = io->sigLogFlushRequested().connect([this]{ flushLog(); });
    }
    return true;
}    


void BodyStateLoggerItem::input()
{
    impl->input();
}


void BodyStateLoggerItem::Impl::input()
{
    if(isOnlineLoggingMode){
        logMutex.lock();
    }
    auto frame = jointEffortBuf->appendFrame();
    for(int i=0; i < numJoints; ++i){
        frame[i] = ioBody->joint(i)->u();
    }
    if(isOnlineLoggingMode){
        logMutex.unlock();
    }
}


void BodyStateLoggerItem::Impl::flushLog()
{
    if(isOnlineLoggingMode){
        logMutex.lock();
    }

    int bufNumFrames = jointEffortBuf->numFrames();
    int logNumFrames = logBodyMotion->numFrames();
    for(int i=0; i < bufNumFrames; ++i){
        auto src = jointEffortBuf->frame(i);
        if(jointEffortLog->numFrames() >= logNumFrames){
            jointEffortLog->popFrontFrame();
        }
        std::copy(src.begin(), src.end(), jointEffortLog->appendFrame().begin());
    }
    jointEffortLog->setOffsetTime(logBodyMotion->offsetTime());

    if(isOnlineLoggingMode){
        logMutex.unlock();
    }
}


void BodyStateLoggerItem::stop()
{
    impl->input();
}


void BodyStateLoggerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Online logging"), impl->isOnlineLoggingModeProperty,
                changeProperty(impl->isOnlineLoggingModeProperty));
    putProperty(_("Joint effort"), true);
}


bool BodyStateLoggerItem::store(Archive& archive)
{
    archive.write("online_logging", impl->isOnlineLoggingModeProperty);

    ListingPtr stateTypes = new Listing;

    if(true){
        stateTypes->append("joint_effort");
    }

    if(!stateTypes->empty()){
        archive.insert("log_state_types", stateTypes);
    }

    return true;
}


bool BodyStateLoggerItem::restore(const Archive& archive)
{
    archive.read("online_logging", impl->isOnlineLoggingModeProperty);

    auto stateTypes = archive.findListing("log_state_types");
    if(stateTypes->isValid()){
        for(auto& node : *stateTypes){
            string type = node->toString();
            if(type == "joint_effort"){

            }
        }
    }

    return true;
}
