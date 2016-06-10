/**
   @brief A component for accessing a joystick control device
*/

#ifndef ConstraintLogRTC_H
#define ConstraintLogRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <fstream>

/*!
 * @class ConstraintLogRTC
 *
 */
class ConstraintLogRTC : public RTC::DataFlowComponentBase
{
public:
    ConstraintLogRTC(RTC::Manager* manager);
    virtual ~ConstraintLogRTC();
    virtual RTC::ReturnCode_t onInitialize();
    virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
    virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

protected:
    // DataInPort declaration
    RTC::TimedDoubleSeq m_larmCF;
    RTC::InPort<RTC::TimedDoubleSeq> m_larmCFIn;
    RTC::TimedDoubleSeq m_rarmCF;
    RTC::InPort<RTC::TimedDoubleSeq> m_rarmCFIn;
  
private:
    std::ofstream outFile;
};


extern "C"
{
    DLL_EXPORT void ConstraintLogRTCInit(RTC::Manager* manager);
};

#endif
