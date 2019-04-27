/**
   @author Shin'ichiro Nakaoka
*/

#include "OpenHRPClockGeneratorItem.h"
#include "corba/OpenHRP/3.1/ClockGenerator.hh"
#include <cnoid/ControllerIO>
#include <cnoid/CorbaUtil>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

MessageView* mv = 0;

class ExecContextInfo : public Referenced
{
public:
    OpenRTM::ExtTrigExecutionContextService_ptr context;
    double period;
    double nextTickTime;

    ExecContextInfo(OpenRTM::ExtTrigExecutionContextService_ptr context, double period)
        : context(context),
          period(period),
          nextTickTime(0.0) {
    }
        
    ~ExecContextInfo() {
        CORBA::release(context);
    }

    bool step(double currentTime) {
        if (currentTime >= nextTickTime){
            try{
                context->tick();
                nextTickTime += period;

            } catch(CORBA::SystemException& ex){
                mv->putln(
                    fmt::format(
                        _("OpenHRP ClockGenerator failed to tick an execution context due to exception: {}."),
                        ex._rep_id()));
                return false;
            }
        }
        return true;
    }

    void reset() {
        nextTickTime = 0.0;
    }

};
typedef ref_ptr<ExecContextInfo> ExecContextInfoPtr;

};

namespace cnoid {
    
class OpenHRPClockGenerator_impl :
        virtual public POA_OpenHRP::ClockGenerator,
        virtual public PortableServer::RefCountServantBase
{
    vector<ExecContextInfoPtr> contexts;
    double currentTime;
        
public:
    OpenHRPClockGenerator_impl() {
        currentTime = 0.0;
    }
        
    ~OpenHRPClockGenerator_impl() {
            
    }
        
    virtual void subscribe(OpenRTM::ExtTrigExecutionContextService_ptr context, ::CORBA::Double period) {
        contexts.push_back(new ExecContextInfo(context, period));
        mv->putln(_("OpenHRP ClockGenerator received a subscription request."));
    }
        
    virtual void unsubscribe(OpenRTM::ExtTrigExecutionContextService_ptr context) {
        for(vector<ExecContextInfoPtr>::iterator p = contexts.begin(); p != contexts.end(); ++p){
            ExecContextInfoPtr& info = *p;
            if(info->context->_is_equivalent(context)){
                contexts.erase(p);
                break;
            }
        }
        mv->putln(_("OpenHRP ClockGenerator received an unsubscription request."));
    }

    void step(double stepTime) {
        vector<ExecContextInfoPtr>::iterator p = contexts.begin();
        while(p != contexts.end()){
            ExecContextInfoPtr& info = *p;
            if(!info->step(currentTime)){
                p = contexts.erase(p);
            } else {
                ++p;
            }
            currentTime += stepTime;
        }
    }

    void reset() {
        vector<ExecContextInfoPtr>::iterator p = contexts.begin();
        while(p != contexts.end()){
            ExecContextInfoPtr& info = *p;
            info->reset();
            ++p;
        }
        currentTime = 0.0;
    }
};
}


OpenHRPClockGenerator_impl* OpenHRPClockGeneratorItem::clockGenerator = 0;


void OpenHRPClockGeneratorItem::initialize(ExtensionManager* ext)
{
    mv = MessageView::instance();
    
    ext->itemManager().registerClass<OpenHRPClockGeneratorItem>(N_("OpenHRPClockGeneratorItem"));
    ext->itemManager().addCreationPanel<OpenHRPClockGeneratorItem>();

    clockGenerator = ext->manage(new OpenHRPClockGenerator_impl());
}


OpenHRPClockGeneratorItem::OpenHRPClockGeneratorItem()
{

}


OpenHRPClockGeneratorItem::OpenHRPClockGeneratorItem(const OpenHRPClockGeneratorItem& org)
    : ControllerItem(org)
{

}


Item* OpenHRPClockGeneratorItem::doDuplicate() const
{
    return new OpenHRPClockGeneratorItem(*this);
}

    
OpenHRPClockGeneratorItem::~OpenHRPClockGeneratorItem()
{

}


void OpenHRPClockGeneratorItem::onDisconnectedFromRoot()
{

}


bool OpenHRPClockGeneratorItem::initialize(ControllerIO* io)
{
    timeStep_ = io->timeStep();
    clockGenerator->reset();
    io->os() << _("OpenHRP ClockGenerator is used for this simulation.") << endl;
    return true;
}


double OpenHRPClockGeneratorItem::timeStep() const
{
    return timeStep_;
}


void OpenHRPClockGeneratorItem::input()
{

}


bool OpenHRPClockGeneratorItem::control()
{
    clockGenerator->step(timeStep_);
    return true;
}


void OpenHRPClockGeneratorItem::output()
{

}


void OpenHRPClockGeneratorItem::stop()
{

}


void OpenHRPClockGeneratorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    ControllerItem::doPutProperties(putProperty);
}


bool OpenHRPClockGeneratorItem::store(Archive& archive)
{
    return ControllerItem::store(archive);
}


bool OpenHRPClockGeneratorItem::restore(const Archive& archive)
{
    return ControllerItem::restore(archive);
}
