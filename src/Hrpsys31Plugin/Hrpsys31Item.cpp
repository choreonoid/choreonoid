/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "Hrpsys31Item.h"
#include "corba/hrpsys/RobotHardwareService.hh"
#include "corba/hrpsys/StateHolderService.hh"
#include <cnoid/ItemManager>
#include <cnoid/RootItem>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <cnoid/CorbaUtil>
#include <cnoid/OpenRTMUtil>
#include <cnoid/Timer>
#include <cnoid/BodyItem>
#include <cnoid/BasicSensors>
#include <cnoid/ExtraBodyStateAccessor>
#include <QMessageBox>
#include <rtm/idl/RTC.hh>
#include <fmt/format.h>
#include <iostream>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;

namespace {

class JointState
{
public:
    JointState() {
        q_actual = 0.0;
        q_target = 0.0;
        power = false;
        servo = false;
        temperature = 0.0;
    }
    double q_actual;
    double q_target;
    bool calib;
    bool power;
    bool servo;
    double temperature;
    int alarm;
};
    
class RobotState : public ExtraBodyStateAccessor
{
public:
    static const int numStateItems = 4;
    static const int numJointStateItems = 7;
    vector<JointState> joints;
    boost::optional<Vector3> localZMP[2];
    double voltage;
    double electricCurrent;
        
    RobotState() {
        voltage = 0.0;
        electricCurrent = 0.0;
    }
        
    virtual int getNumStateItems() const {
        return numStateItems;
    }
    virtual int getNumJointStateItems() const {
        return numJointStateItems;
    }
    virtual const char* getStateItemName(int stateIndex) const {
        switch(stateIndex){
        case 0: return N_("R-Foot ZMP");
        case 1: return N_("L-Foot ZMP");
        case 2: return N_("Voltage");
        case 3: return N_("Current");
        default: return "";
        }
    }
        
    virtual const char* getStateItemLabel(int stateIndex) const {
        return dgettext(CNOID_GETTEXT_DOMAIN_NAME, RobotState::getStateItemName(stateIndex));
    }
        
    virtual const char* getJointStateItemName(int jointStateIndex) const {
        switch(jointStateIndex){
        case 0: return N_("Pos");
        case 1: return N_("Target");
        case 2: return N_("Calib");
        case 3: return N_("Power");
        case 4: return N_("Servo");
        case 5: return N_("Temp");
        case 6: return N_("Alarm");
        default: return "";
        }
    }
        
    virtual const char* getJointStateItemLabel(int stateIndex) const {
        return dgettext(CNOID_GETTEXT_DOMAIN_NAME, RobotState::getJointStateItemName(stateIndex));
    }
        
    virtual void getState(std::vector<Value>& out_state) const {
        out_state.resize(numStateItems);
        for(int i=0; i < 2; ++i){
            if(localZMP[i]){
                out_state[i+0] = *localZMP[i];
            } else {
                out_state[i+0] = ExtraBodyStateAccessor::none;
            }
        }
        out_state[2] = voltage;
        out_state[3] = electricCurrent;
    }
    virtual void getJointState(Array2D<Value>& out_jointState) const {
        const int nj = joints.size();
        out_jointState.resize(nj, numJointStateItems);
        for(int i=0; i < nj; ++i){
            const JointState& js = joints[i];
            Array2D<Value>::Row jsout = out_jointState.row(i);
            jsout[0] = ExtraBodyStateAccessor::Angle(js.q_actual);
            jsout[1] = ExtraBodyStateAccessor::Angle(js.q_target);
            jsout[2] = js.calib;
            jsout[3] = js.power;
            jsout[4] = js.servo;
            jsout[5] = js.temperature;
            if(js.temperature >= 60.0){
                jsout[5].setAttribute(ExtraBodyStateAccessor::STRONG_WARNING);
            } else if(js.temperature >= 59.0){
                jsout[5].setAttribute(ExtraBodyStateAccessor::WARNING);
            }
            jsout[6] = js.alarm;
        }
    }
};
    
typedef ref_ptr<RobotState> RobotStatePtr;
}       


namespace cnoid {
    
class Hrpsys31ItemImpl
{
public:
    Hrpsys31Item* self;
    MessageView* mv;
        
    string host;
    int port;
    NamingContextHelper naming;

    string robotHardwareName;
    OpenHRP::RobotHardwareService_var robotHardwareService;
        
    string stateHolderName;
    OpenHRP::StateHolderService_var stateHolderService;

    RobotStatePtr robotState;

    BodyItemPtr bodyItem;

    enum JointUpdateMode { UPDATE_BY_ACTUAL, UPDATE_BY_REFERENCE, UPDATE_MODE_SIZE };
    Selection bodyJointUpdateMode;

    ForceSensorPtr footForceSensors[2];
    double verticalForceThreshForFootContact;

    bool isStateReadingEnabled;
    double readInterval; // [ms]
    bool doConnectOnLoading;
        
    Timer timer;
        
    Hrpsys31ItemImpl(Hrpsys31Item* self);
    Hrpsys31ItemImpl(Hrpsys31Item* self, const Hrpsys31ItemImpl& org);
    ~Hrpsys31ItemImpl();
    void init();
    void onPositionChanged();
    bool connectToRobot();
    RTC::RTObject_ptr findRTC(const string& rtcName);
    template<class ServiceType>
    typename ServiceType::_ptr_type findService(RTC::RTObject_ptr rtc, const string& serviceName);
    bool disconnectFromRobot();
    bool activateServos(bool on);
    void onReadRequest();
    template<class SensorType, class SequenceType>
    void copySensorState(DeviceList<SensorType>& sensors, SequenceType& seq);
    void doPutProperties(PutPropertyFunction& putProperty);
    bool onReadIntervalEdited(int interval);
    bool store(Archive& archive);
    bool restore(const Archive& archive);
};
}


void Hrpsys31Item::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;
    if(!initialized){
        ItemManager& im = ext->itemManager();
        im.registerClass<Hrpsys31Item>(N_("Hrpsys31Item"));
        im.addCreationPanel<Hrpsys31Item>();
    }
}


Hrpsys31Item::Hrpsys31Item()
{
    impl = new Hrpsys31ItemImpl(this);
}


Hrpsys31ItemImpl::Hrpsys31ItemImpl(Hrpsys31Item* self)
    : self(self),
      bodyJointUpdateMode(UPDATE_MODE_SIZE, CNOID_GETTEXT_DOMAIN_NAME)
{
    host = "localhost";
    port = 2809;
    robotHardwareName = "RobotHardware0";
    stateHolderName = "StateHolder0";
    isStateReadingEnabled = true;
    readInterval = 200.0;
    doConnectOnLoading = false;

    bodyJointUpdateMode.setSymbol(UPDATE_BY_ACTUAL, N_("Actual"));
    bodyJointUpdateMode.setSymbol(UPDATE_BY_REFERENCE, N_("Reference"));
    bodyJointUpdateMode.select(UPDATE_BY_ACTUAL);

    init();
}


Hrpsys31Item::Hrpsys31Item(const Hrpsys31Item& org)
    : RobotAccessItem(org)
{
    impl = new Hrpsys31ItemImpl(this, *org.impl);
}


Hrpsys31ItemImpl::Hrpsys31ItemImpl(Hrpsys31Item* self, const Hrpsys31ItemImpl& org)
    : self(self)
{
    host = org.host;
    port = org.port;
    robotHardwareName = org.robotHardwareName;
    stateHolderName = org.stateHolderName;
    bodyJointUpdateMode = org.bodyJointUpdateMode;
    isStateReadingEnabled = org.isStateReadingEnabled;
    readInterval = org.readInterval;
    doConnectOnLoading = org.doConnectOnLoading;

    init();
}


void Hrpsys31ItemImpl::init()
{
    mv = MessageView::instance();
    timer.sigTimeout().connect(std::bind(&Hrpsys31ItemImpl::onReadRequest, this));
}


Hrpsys31Item::~Hrpsys31Item()
{
    delete impl;
}


Hrpsys31ItemImpl::~Hrpsys31ItemImpl()
{
    disconnectFromRobot();
}


void Hrpsys31Item::onPositionChanged()
{
    impl->onPositionChanged();
}


void Hrpsys31ItemImpl::onPositionChanged()
{
    disconnectFromRobot();
    
    BodyItem* targetBodyItem = self->findOwnerItem<BodyItem>();
    if(targetBodyItem != bodyItem){
        bodyItem = targetBodyItem;
        if(bodyItem){
            BodyPtr body = bodyItem->body();
            robotState = body->getOrCreateCache<RobotState>("Hrpsys31RobotState");
            robotState->joints.resize(body->numJoints());

            // get the foot force sensors of a biped robot
            const Mapping& info = *body->info();
            footForceSensors[0] = body->findDevice<ForceSensor>(info.get("rightFootForceSensor", ""));
            footForceSensors[1] = body->findDevice<ForceSensor>(info.get("leftFootForceSensor", ""));
            info.read("verticalForceThreshForFootContact", verticalForceThreshForFootContact);
                
        } else {
            robotState = 0;
        }
    }
}


bool Hrpsys31Item::connectToRobot()
{
    return impl->connectToRobot();
}


bool Hrpsys31ItemImpl::connectToRobot()
{
    bool result = false;

    disconnectFromRobot();

    mv->putln(format(_("Connecting {} to the target robot ..."), self->name()));
    mv->flush();

    naming.setLocation(host, port);

    if(!naming.isAlive()){
        mv->putln(format(_("Nameserver is not found at {0}:{1}."), host, port));

    } else {
        RTC::RTObject_var robotHardware = findRTC(robotHardwareName);
        if(!CORBA::is_nil(robotHardware)){
            robotHardwareService = findService<OpenHRP::RobotHardwareService>(robotHardware, "RobotHardwareService");
            // activate the execution context of the robot hardware component
            if(!CORBA::is_nil(robotHardwareService)){
                RTC::ExecutionContextList_var eclist = robotHardware->get_owned_contexts();
                if(eclist->length() > 0){
                    RTC::ExecutionContext_ptr ec = eclist[0];
                    if(!CORBA::is_nil(ec)){
                        if(ec->get_component_state(robotHardware) != RTC::ACTIVE_STATE){
                            ec->activate_component(robotHardware);
                        }
                    }
                }
            }
        }

        RTC::RTObject_var stateHolder = findRTC(stateHolderName);
        if(!CORBA::is_nil(stateHolder)){
            stateHolderService = findService<OpenHRP::StateHolderService>(stateHolder, "StateHolderService");
        }

        result = (!CORBA::is_nil(robotHardwareService) || !CORBA::is_nil(stateHolderService));
    }

    if(result){
        mv->putln("Connected.");
        if(bodyItem && isStateReadingEnabled){
            timer.setInterval(readInterval);
            timer.start();
            mv->putln("Periodic state reading has been started.");
        }
    }

    return result;
}


RTC::RTObject_ptr Hrpsys31ItemImpl::findRTC(const string& rtcName)
{
    RTC::RTObject_ptr rtc = naming.findObject<RTC::RTObject>(rtcName, "rtc");
    if(naming.isObjectAlive(rtc)){
        mv->putln(format(_("Component \"{}\" has been found."), rtcName));
    } else {
        mv->putln(format(_("Component \"{}\" is not found."), rtcName));
        rtc = RTC::RTObject::_nil();
    }
    return rtc;
}


template<class ServiceType>
typename ServiceType::_ptr_type Hrpsys31ItemImpl::findService(RTC::RTObject_ptr rtc, const string& serviceName)
{
    typename ServiceType::_ptr_type service = findRTCService<ServiceType>(rtc, serviceName);
    if(naming.isObjectAlive(service)){
        mv->putln(format(_("Service port \"{}\" has been found."), serviceName));
    } else {
        mv->putln(format(_("Service port \"{}\" is not found."), serviceName));
        service = ServiceType::_nil();
    }
    return service;
}


void Hrpsys31Item::onDisconnectedFromRoot()
{
    impl->disconnectFromRobot();
}


bool Hrpsys31Item::disconnectFromRobot()
{
    return impl->disconnectFromRobot();
}


bool Hrpsys31ItemImpl::disconnectFromRobot()
{
    if(timer.isActive()){
        timer.stop();
    }
    bool disconnected = false;
    if(!CORBA::is_nil(robotHardwareService)){
        robotHardwareService = OpenHRP::RobotHardwareService::_nil();
        disconnected = true;
    }
    if(!CORBA::is_nil(stateHolderService)){
        stateHolderService = OpenHRP::StateHolderService::_nil();
        disconnected = true;
    }
    if(disconnected){
        mv->putln(format(_("{} is disconnected from the target robot"), self->name()));
    }
    return disconnected;
}


bool Hrpsys31Item::activateServos(bool on)
{
    return impl->activateServos(on);
}


bool Hrpsys31ItemImpl::activateServos(bool on)
{
    bool confirmed = true;
    bool result = false;
    
    if(CORBA::is_nil(robotHardwareService)){
        mv->putln(format(_("{0} cannot turn on / off the servos because it is not connected with {1}."),
                         self->name(), robotHardwareName));
    } else {
        OpenHRP::RobotHardwareService::RobotState_var state;
        try {
            robotHardwareService->getStatus(state);
            const OpenHRP::RobotHardwareService::LongSequenceSequence& ss = state->servoState;
            bool isOperationValid = false;
            for(CORBA::ULong i=0; i < ss.length(); ++i){
                bool servo = (ss[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK);
                if((on && !servo) || (!on && servo)){
                    isOperationValid = true;
                    break;
                }
            }
            if(!isOperationValid){
                mv->putln(format(_("All the target servos of {} have already been turned on."), self->name()));
                result = true;
            } else {
                confirmed = showConfirmDialog((on ? _("Servo On") : _("Servo Off")), _("Click OK to continue."));

                if(confirmed){
                    result = robotHardwareService->servo(
                        "all", (on ? OpenHRP::RobotHardwareService::SWITCH_ON : OpenHRP::RobotHardwareService::SWITCH_OFF));
                    if(result){
                        mv->putln(format(_("The target servos of {} have been turned on."), self->name()));
                        onReadRequest();
                    }
                }
            }
        } catch(CORBA::Exception& ex){
            mv->putln(format(_("{}: A CORBA Exception happened."), self->name()));
        }
        if(confirmed && !result){
            mv->putln(format(_("{} cannot turn on the servos."), self->name()));
        }
    }
    return result;
}


bool Hrpsys31Item::setStateReadingEnabled(bool on)
{
    return true;
}


void Hrpsys31ItemImpl::onReadRequest()
{
    if(!bodyItem){
        return;
    }
    
    bool jointStateChanged = false;
    bool robotStateChanged = false;
    const BodyPtr& body = bodyItem->body();
        
    if(!CORBA::is_nil(robotHardwareService)){
        bool connectionFailed = false;
        OpenHRP::RobotHardwareService::RobotState_var state;
        try {
            robotHardwareService->getStatus(state);
            
        } catch(CORBA::Exception& ex){
            robotHardwareService = OpenHRP::RobotHardwareService::_nil();
            mv->putln(format(_("{0}: Access to {1} failed. The connection is terminated."),
                    self->name(), robotHardwareName));
            connectionFailed = true;
        }
        
        if(!connectionFailed){
            
            const int n = std::min(body->numJoints(), (int)state->angle.length());
            if(n > 0){
                for(int i=0; i < n; ++i){
                    JointState& js = robotState->joints[i];
                    Link* joint = body->joint(i);
                    if(bodyJointUpdateMode.is(UPDATE_BY_ACTUAL)){
                        joint->q() = state->angle[i];
                    } else {
                        joint->q() = state->command[i];
                    }
                    joint->u() = state->torque[i];
                    js.q_actual = state->angle[i];
                    js.q_target = state->command[i];
                    CORBA::Long s = state->servoState[i][0];
                    js.calib = s & OpenHRP::RobotHardwareService::CALIB_STATE_MASK;
                    js.servo = s & OpenHRP::RobotHardwareService::SERVO_STATE_MASK;
                    js.power = s & OpenHRP::RobotHardwareService::POWER_STATE_MASK;
                    js.temperature = (s & OpenHRP::RobotHardwareService::DRIVER_TEMP_MASK)
                        >> OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT; 
                    js.alarm = (s & OpenHRP::RobotHardwareService::SERVO_ALARM_MASK)
                        >> OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT; 
                }
                jointStateChanged = true;
            }

            DeviceList<ForceSensor> forceSensors = body->devices<ForceSensor>();
            copySensorState(forceSensors, state->force);
            DeviceList<RateGyroSensor> gyros = body->devices<RateGyroSensor>();
            copySensorState(gyros, state->rateGyro);
            DeviceList<AccelerationSensor> accelSensors = body->devices<AccelerationSensor>();
            copySensorState(accelSensors, state->accel);

            for(int i=0; i < 2; ++i){
                ForceSensorPtr& s = footForceSensors[i];
                if(!s || s->f().z() < verticalForceThreshForFootContact){
                    robotState->localZMP[i] = boost::none;
                } else {
                    robotState->localZMP[i] =
                        Vector3(-s->tau().y() / s->f().z(), s->tau().x() / s->f().z(), 0.0);
                }
            }

            robotState->voltage = state->voltage;
            robotState->electricCurrent = state->current;
            robotStateChanged = true;
        }
    }
    
    if(!CORBA::is_nil(stateHolderService)){
        bool connectionFailed = false;
        OpenHRP::StateHolderService::Command_var command;
        try {
            stateHolderService->getCommand(command);
            
        } catch(CORBA::Exception& ex){
            stateHolderService = OpenHRP::StateHolderService::_nil();
            mv->putln(format(_("{0}: Access to {1} failed. The connection is terminated."),
                             self->name(), stateHolderName));
            connectionFailed = true;
        }
        
        if(!connectionFailed){
            const int n = std::min(body->numJoints(), (int)command->jointRefs.length());
            if(n > 0){
                for(int i=0; i < n; ++i){
                    const double q = command->jointRefs[i];
                    if(bodyJointUpdateMode.is(UPDATE_BY_REFERENCE)){
                        body->joint(i)->q() = q;
                        jointStateChanged = true;
                    }
                    robotState->joints[i].q_target = q;
                }
            }
            if(command->zmp.length() == 3){
                bodyItem->setZmp(Eigen::Map<const Vector3>(&command->zmp[0]));
                robotStateChanged = true;
            }
        }
    }
        
    if(jointStateChanged){
        bodyItem->notifyKinematicStateChangeLater(true);
    }
    if(robotStateChanged){
        robotState->notifyStateChange();
    }

}


template<class SensorType, class SequenceType>
void Hrpsys31ItemImpl::copySensorState(DeviceList<SensorType>& sensors, SequenceType& seq)
{
    if(seq.length() > 0){
        const int n = std::min((int)sensors.size(), (int)seq.length());
        for(int i=0; i < n; ++i){
            SensorType* sensor = sensors[i];
            sensor->readState(&seq[i][0]);
            sensor->notifyStateChange();
        }
    }
}


bool Hrpsys31Item::sendCurrentPose()
{
    return true;
}


bool Hrpsys31Item::setPlaybackSyncEnabled(bool on)
{
    return true;
}


Item* Hrpsys31Item::doDuplicate() const
{
    return new Hrpsys31Item(*this);
}


void Hrpsys31Item::doPutProperties(PutPropertyFunction& putProperty)
{
    RobotAccessItem::doPutProperties(putProperty);
    impl->doPutProperties(putProperty);
}


void Hrpsys31ItemImpl::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Host"), host, changeProperty(host));
    putProperty(_("Port"), port, changeProperty(port));
    putProperty(_("RobotHardware"), robotHardwareName, changeProperty(robotHardwareName));
    putProperty(_("StateHolder"), stateHolderName, changeProperty(stateHolderName));
    putProperty(_("State reading"), isStateReadingEnabled,
                std::bind(&Hrpsys31Item::setStateReadingEnabled, self, _1));
    putProperty(_("Read inverval"), (int)readInterval,
                std::bind(&Hrpsys31ItemImpl::onReadIntervalEdited, this, _1));
    putProperty(_("Body update mode"), bodyJointUpdateMode,
                std::bind((bool(Selection::*)(int))&Selection::select, &bodyJointUpdateMode, _1));
    putProperty(_("Connect on loading"), doConnectOnLoading, changeProperty(doConnectOnLoading));
}


bool Hrpsys31ItemImpl::onReadIntervalEdited(int interval)
{
    if(interval > 0.0){
        readInterval = interval;
        return true;
    }
    return false;
}


bool Hrpsys31Item::store(Archive& archive)
{
    if(!RobotAccessItem::store(archive)){
        return false;
    }
    return impl->store(archive);
}


bool Hrpsys31ItemImpl::store(Archive& archive)
{
    archive.write("host", host);
    archive.write("port", port);
    archive.write("RobotHardware", robotHardwareName);
    archive.write("StateHolder", stateHolderName);
    archive.write("stateReading", isStateReadingEnabled);
    archive.write("readInterval", readInterval);
    archive.write("bodyUpdateMode", bodyJointUpdateMode.selectedSymbol());
    archive.write("connectOnLoading", doConnectOnLoading);
    return true;
}


bool Hrpsys31Item::restore(const Archive& archive)
{
    if(!RobotAccessItem::restore(archive)){
        return false;
    }
    return impl->restore(archive);
}


bool Hrpsys31ItemImpl::restore(const Archive& archive)
{
    archive.read("host", host);
    archive.read("port", port);
    archive.read("RobotHardware", robotHardwareName);
    archive.read("StateHolder", stateHolderName);
    archive.read("stateReading", isStateReadingEnabled);
    archive.read("readInterval", readInterval);
    string symbol;
    if(archive.read("bodyUpdateMode", symbol)){
        bodyJointUpdateMode.select(symbol);
    }
    if(archive.read("connectOnLoading", doConnectOnLoading)){
        if(doConnectOnLoading){
            // connect here
        }
    }
    return true;
}
