/**
   @brief A component for accessing a joystick control device
*/

#ifndef JoystickRTC_H
#define JoystickRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <cnoid/Joystick>

/*!
 * @class JoystickRTC
 * @brief Access a joystick control device.
 *
 */
class JoystickRTC : public RTC::DataFlowComponentBase
{
public:
    JoystickRTC(RTC::Manager* manager);
    virtual ~JoystickRTC();
    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataOutPort declaration
    RTC::TimedFloatSeq m_axes;
    /*!
     * Values of the joystick axes.
     * - Type: TimedFloatSeq
     * - Number: Variable.
     * - Semantics: Axes values normalised to between -1 and 1, with 0 as the cent
     *              re.
     */
    RTC::OutPort<RTC::TimedFloatSeq> m_axesOut;
    RTC::TimedBooleanSeq m_buttons;
    /*!
     * Joystick button values.
     * - Type: TimedBooleanSeq
     * - Number: Variable
     * - Semantics: True if a button is pressed, false otherwise.
     */
    RTC::OutPort<RTC::TimedBooleanSeq> m_buttonsOut;
  
private:
    cnoid::Joystick* m_joystick; 
    std::string m_device;
    unsigned int m_debugLevel;
};


extern "C"
{
    DLL_EXPORT void JoystickRTCInit(RTC::Manager* manager);
};

#endif
