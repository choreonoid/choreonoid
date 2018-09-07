/**
   @author Japan Atomic Energy Agency
*/


#ifndef MULTICOPTERCONTROLLERRTC_H
#define MULTICOPTERCONTROLLERRTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

using namespace RTC;

class QuadcopterControllerRTC
  : public RTC::DataFlowComponentBase
{
 public:
  QuadcopterControllerRTC(RTC::Manager* manager);
  ~QuadcopterControllerRTC();

   virtual RTC::ReturnCode_t onInitialize();
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
  RTC::TimedFloatSeq m_axes;

  InPort<RTC::TimedFloatSeq> m_axesIn;
  RTC::TimedBooleanSeq m_buttons;

  InPort<RTC::TimedBooleanSeq> m_buttonsIn;
  RTC::TimedDoubleSeq m_zrpy;

  InPort<RTC::TimedDoubleSeq> m_zrpyIn;
  RTC::TimedDoubleSeq m_q;

  InPort<RTC::TimedDoubleSeq> m_qIn;

  RTC::TimedDoubleSeq m_force;

  OutPort<RTC::TimedDoubleSeq> m_forceOut;
  RTC::TimedDoubleSeq m_torque;

  OutPort<RTC::TimedDoubleSeq> m_torqueOut;
  RTC::TimedDoubleSeq m_u;

  OutPort<RTC::TimedDoubleSeq> m_uOut;

 private:
  double zrpyref[4];
  double zrpyprev[4];
  double dzrpyref[4];
  double dzrpyprev[4];
  double f[4];
  double qref;
  double qprev;
  std::vector<float> lastAxes;
  bool rotorswitch;
  bool power;
  bool powerprev;

};


extern "C"
{
  DLL_EXPORT void QuadcopterControllerRTCInit(RTC::Manager* manager);
};

#endif
