/**
   @author Shin'ichiro Nakaoka
*/

#include "BodyIoRTCItem.h"
#include <cnoid/BodyIoRTC>
#include <cnoid/BodyItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

#include "LoggerUtil.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class BodyIoRTCItemImpl : public ControllerIO
{
public:
    BodyIoRTCItem* self;
    BodyItem* bodyItem;
    BodyIoRTC* bodyIoRTC;
    ControllerIO* io;
    MessageView* mv;

    BodyIoRTCItemImpl(BodyIoRTCItem* self);
    BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org);
    void setBodyItem(BodyItem* newBodyItem, bool forceReset);
    bool createBodyIoRTC();

    // Virtual functions of ControllerIO
    virtual Body* body() override;
    virtual std::string optionString() const override;
    virtual std::ostream& os() const override;
    virtual double timeStep() const override;
    virtual double currentTime() const override;
    virtual bool isNoDelayMode() const override;
    virtual bool setNoDelayMode(bool on) override;
};

}

void BodyIoRTCItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ext->itemManager().registerClass<BodyIoRTCItem>(N_("BodyIoRTCItem"));
        ext->itemManager().addCreationPanel<BodyIoRTCItem>();
        initialized = true;
    }
}


BodyIoRTCItem::BodyIoRTCItem()
{
    impl = new BodyIoRTCItemImpl(this);
    useOnlySimulationExecutionContext();
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self)
    : self(self),
      mv(MessageView::instance())
{
    bodyItem = nullptr;
    bodyIoRTC = nullptr;
    io = nullptr;
}


BodyIoRTCItem::BodyIoRTCItem(const BodyIoRTCItem& org)
    : ControllerRTCItem(org)
{
    impl = new BodyIoRTCItemImpl(this, *org.impl);
    useOnlySimulationExecutionContext();
}


BodyIoRTCItemImpl::BodyIoRTCItemImpl(BodyIoRTCItem* self, const BodyIoRTCItemImpl& org)
    : BodyIoRTCItemImpl(self)
{

}


BodyIoRTCItem::~BodyIoRTCItem()
{
    delete impl;
}


Item* BodyIoRTCItem::doDuplicate() const
{
    return new BodyIoRTCItem(*this);
}


void BodyIoRTCItem::onConnectedToRoot()
{
    /**
       Do nothing here to cancel the implementation of ControllerRTCItem.
       The RTC is created when the onPositionChanged function is called.
    */
}


void BodyIoRTCItem::onPositionChanged()
{
    impl->setBodyItem(findOwnerItem<BodyItem>(), false);
}


void BodyIoRTCItem::onOptionsChanged()
{
    // Recreate RTC with new options
    impl->createBodyIoRTC();
}


void BodyIoRTCItemImpl::setBodyItem(BodyItem* newBodyItem, bool forceReset)
{
    if(newBodyItem != bodyItem || forceReset){
        bodyItem = newBodyItem;
        self->createRTC();
    }
}


std::string BodyIoRTCItem::getDefaultRTCInstanceName() const
{
    return impl->bodyItem->name() + "-" + rtcModuleName();
}


Body* BodyIoRTCItemImpl::body()
{
    if(io){
        return io->body();
    } else if(bodyItem){
        return bodyItem->body();
    }
    return nullptr;
}


std::string BodyIoRTCItemImpl::optionString() const
{
    if(io){
        return getIntegratedOptionString(io->optionString(), self->optionString());
    }
    return self->optionString();
}


std::ostream& BodyIoRTCItemImpl::os() const
{
    return mv->cout();
}


double BodyIoRTCItemImpl::timeStep() const
{
    return io ? io->timeStep() : 0.0;
}
        

double BodyIoRTCItemImpl::currentTime() const
{
    return io ? io->currentTime() : 0.0;
}
        

bool BodyIoRTCItemImpl::isNoDelayMode() const
{
    return self->isNoDelayMode();
}
        

bool BodyIoRTCItemImpl::setNoDelayMode(bool on)
{
    return self->setNoDelayMode(on);
}
        

bool BodyIoRTCItem::createRTC()
{
    return impl->createBodyIoRTC();
}


bool BodyIoRTCItemImpl::createBodyIoRTC()
{
  DDEBUG("BodyIoRTCItemImpl::createBodyIoRTC");
    if(!bodyItem){
        self->deleteRTC(true);
        return false;
    }
    
    if(self->createRTCmain(true)){

        bodyIoRTC = dynamic_cast<BodyIoRTC*>(self->rtc());
        if(!bodyIoRTC){
            mv->putln(
                format(_("RTC \"{0}\" of {1} cannot be used as a BodyIoRTC because it is not derived from it."),
                       self->rtcModuleName(), self->name()),
                MessageView::ERROR);
            self->deleteRTC(false);
            return false;
        }
        bool initialized = false;
        if(bodyIoRTC->onInitialize(bodyItem->body()) == RTC::RTC_OK){ // old API
            initialized = true;
        } else {
            initialized = bodyIoRTC->initializeIO(this);
        }

        if(!initialized){
            mv->putln(
                format(_("RTC \"{0}\" of {1} failed to initialize."),
                       self->rtcModuleName(), self->name()),
                MessageView::ERROR);
            self->deleteRTC(true);
            return false;
        }
        
        mv->putln(format(_("BodyIoRTC \"{}\" has been created."), self->rtcInstanceName()));
        
        return true;
    }

    return false;
}


void BodyIoRTCItem::deleteRTC(bool waitToBeDeleted)
{
    ControllerRTCItem::deleteRTC(waitToBeDeleted);
    impl->bodyIoRTC = 0;
}


bool BodyIoRTCItem::initialize(ControllerIO* io)
{
    if(impl->bodyIoRTC){
        impl->io = io;
        return impl->bodyIoRTC->initializeSimulation(impl);
    }
    return false;
}


bool BodyIoRTCItem::start()
{
    if(impl->bodyIoRTC){
        if(impl->bodyIoRTC->startSimulation()){
            if(ControllerRTCItem::start()){
                impl->bodyIoRTC->inputFromSimulator();
                return true;
            }
        }
    }
    return false;
}


void BodyIoRTCItem::input()
{
    impl->bodyIoRTC->inputFromSimulator();
}


void BodyIoRTCItem::output()
{
    impl->bodyIoRTC->outputToSimulator();
}


void BodyIoRTCItem::stop()
{
    impl->bodyIoRTC->stopSimulation();
    ControllerRTCItem::stop();
    impl->io = nullptr;
}
