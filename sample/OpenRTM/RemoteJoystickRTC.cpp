/**
   Ext joystick obtaining a remote joystick state via RTC
   @author Shin'ichiro Nakaoka
*/

#include <cnoid/ExtJoystick>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/DataInPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>

using namespace cnoid;

class RemoteJoystickRTC : public RTC::DataFlowComponentBase, public ExtJoystick
{
public:
    RTC::TimedFloatSeq axes;
    RTC::InPort<RTC::TimedFloatSeq> axesIn;
    Signal<void(int id, double position)> sigAxis_;
  
    RTC::TimedBooleanSeq buttons;
    RTC::InPort<RTC::TimedBooleanSeq> buttonsIn;
    Signal<void(int id, bool isPressed)> sigButton_;
    
    RemoteJoystickRTC(RTC::Manager* manager)
        : RTC::DataFlowComponentBase(manager),
          axesIn("axes", axes),
          buttonsIn("buttons", buttons)
    {
        ExtJoystick::registerJoystick("RemoteJoystickRTC", this);
    }

    virtual ~RemoteJoystickRTC()
    {
        // ExtJoystick::unregisterJoystick(this);
    }
    
    virtual RTC::ReturnCode_t onInitialize() override
    {
        addInPort("axes", axesIn);
        addInPort("buttons", buttonsIn);

        return RTC::RTC_OK;
    }
    
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id) override
    {
        return RTC::RTC_OK;
    }

    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id) override
    {
        return RTC::RTC_OK;
    }
    
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id) override
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

    virtual bool isActive() const override
    {
        return true; // TODO: check the RTC state
    }
    
    virtual SignalProxy<void(int id, double position)> sigAxis() override
    {
        return sigAxis_;
    }
    
    virtual SignalProxy<void(int id, bool isPressed)> sigButton() override
    {
        return sigButton_;
    }
};

extern "C"
{
    DLL_EXPORT void RemoteJoystickRTCInit(RTC::Manager* manager)
    {
        static const char* spec[] = {
            "implementation_id", "RemoteJoystickRTC",
            "type_name",         "RemoteJoystickRTC",
            "description",       "Ext joystick obtaining a remote joystick state via RTC",
            "version",           "1.0.0",
            "vendor",            "AIST",
            "category",          "HumanInterfaceDevice",
            "activity_type",     "DataFlowComponent",
            "max_instance",      "1",
            "language",          "C++",
            "lang_type",         "compile",
            ""
        };
        coil::Properties profile(spec);
        
        manager->registerFactory(
            profile, RTC::Create<RemoteJoystickRTC>, RTC::Delete<RemoteJoystickRTC>);
    }
};
