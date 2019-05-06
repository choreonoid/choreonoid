/**
   A simple controller that inputs operation commands from a remote joystick via RTC
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/SimpleController>
#include <cnoid/SharedJoystick>
#include <cnoid/OpenRTMUtil>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <fmt/format.h>

using namespace cnoid;

class JoystickInputRTC : public RTC::DataFlowComponentBase, public JoystickInterface
{
public:
    RTC::TimedFloatSeq axes;
    RTC::InPort<RTC::TimedFloatSeq> axesIn;
  
    RTC::TimedBooleanSeq buttons;
    RTC::InPort<RTC::TimedBooleanSeq> buttonsIn;

    JoystickInputRTC(RTC::Manager* manager)
        : RTC::DataFlowComponentBase(manager),
          axesIn("axes", axes),
          buttonsIn("buttons", buttons)
    {

    }

    virtual RTC::ReturnCode_t onInitialize()
    {
        addInPort("axes", axesIn);
        addInPort("buttons", buttonsIn);
        return RTC::RTC_OK;
    }
        
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id)
    {
        if(axesIn.isNew()){
            axesIn.read();
        }
        if(buttonsIn.isNew()){
            buttonsIn.read();
        }
        return RTC::RTC_OK;
    }

    virtual int numAxes() const override
    {
        return axes.data.length();
    }
    
    virtual int numButtons() const override
    {
        return buttons.data.length();
    }

    virtual bool readCurrentState() override
    {
        return true;
    }

    virtual double getPosition(int axis) const override
    {
        if(axis < axes.data.length()){
            return axes.data[axis];
        }
        return 0.0;
    }

    virtual bool getButtonState(int button) const override
    {
        if(button < buttons.data.length()){
            return buttons.data[button];
        }
        return false;
    }
};


class RemoteJoystickInputController : public SimpleController
{
    SharedJoystick* joystick;
    JoystickInputRTC* joystickInputRTC;
    OpenRTM::ExtTrigExecutionContextService_var execContext;
    
public:
    RemoteJoystickInputController()
    {
        static const char* spec[] = {
            "implementation_id", "JoystickInput",
            "type_name",         "JoystickInput",
            "description",       "RTC for inputting joystick state via RTC",
            "version",           "1.0.0",
            "vendor",            "AIST",
            "category",          "HumanInterfaceDevice",
            "activity_type",     "PERIODIC",
            "kind",              "DataFlowComponent",
            "max_instance",      "1",
            "language",          "C++",
            "lang_type",         "compile",
            ""
        };
        coil::Properties profile(spec);
        RTC::Manager::instance().registerFactory(
            profile, RTC::Create<JoystickInputRTC>, RTC::Delete<JoystickInputRTC>);

        joystickInputRTC = 0;
        execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
    }

    ~RemoteJoystickInputController()
    {
        if(joystickInputRTC){
            execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
            cnoid::deleteRTC(joystickInputRTC);
        }
    }
    
    bool configure(SimpleControllerConfig* config) override
    {
        if(joystickInputRTC){
            execContext = OpenRTM::ExtTrigExecutionContextService::_nil();
            cnoid::deleteRTC(joystickInputRTC);
            joystickInputRTC = 0;
        }
        
#if defined(OPENRTM_VERSION11)
        std::string args =
            fmt::format(
                "JoystickInput?instance_name={}&exec_cxt.periodic.type=ChoreonoidExecutionContext",
                config->controllerName());
#elif defined(OPENRTM_VERSION12)
        std::string args =
            fmt::format(
                "JoystickInput?instance_name={}&execution_contexts=SimulationExecutionContext&exec_cxt.periodic.type=ChoreonoidExecutionContext",
                config->controllerName());
#endif
            
        joystickInputRTC = dynamic_cast<JoystickInputRTC*>(cnoid::createManagedRTC(args));

        if(joystickInputRTC){
            RTC::ExecutionContextList_var eclist = joystickInputRTC->get_owned_contexts();
            for(CORBA::ULong i=0; i < eclist->length(); ++i){
                if(!CORBA::is_nil(eclist[i])){
                    execContext = OpenRTM::ExtTrigExecutionContextService::_narrow(eclist[i]);
                    break;
                }
            }
        }

        return !CORBA::is_nil(execContext);
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        joystick = io->getOrCreateSharedObject<SharedJoystick>("joystick");
        joystick->setJoystick(joystickInputRTC);
        return true;
    }

    virtual bool start() override
    {
        if(execContext->activate_component(joystickInputRTC->getObjRef()) == RTC::RTC_OK){
            execContext->tick();
            return true;
        }
        return false;
    }

    virtual bool control() override
    {
        execContext->tick();
        return false;
    }

    virtual void stop() override
    {
        if(execContext->deactivate_component(joystickInputRTC->getObjRef()) == RTC::RTC_OK){
            execContext->tick();
        }
    }
};


CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RemoteJoystickInputController)
