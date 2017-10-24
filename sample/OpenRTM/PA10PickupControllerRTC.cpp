/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "PA10PickupControllerRTC.h"
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

namespace {

const double TIMESTEP = 0.001;

const double pgain[] = {
    35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0, 35000.0,
    17000.0, 17000.0 };

const double dgain[] = {
    220.0, 220.0, 220.0, 220.0, 220.0, 220.0, 220.0,
    220.0, 220.0 };

const char* samplepd_spec[] =
{
    "implementation_id", "PA10PickupControllerRTC",
    "type_name",         "PA10PickupControllerRTC",
    "description",       "OpenRTM_PA10 Pickup Controller component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};

Vector3 toRadianVector3(double x, double y, double z){
    return Vector3(radian(x), radian(y), radian(z));
}
}

    
PA10PickupControllerRTC::PA10PickupControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_torqueOut("u_out", m_torque_out)
{

}


PA10PickupControllerRTC::~PA10PickupControllerRTC()
{

}


RTC::ReturnCode_t PA10PickupControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);
  
    // Set OutPort buffer
    addOutPort("u_out", m_torqueOut);

    string modelfile = getNativePathString(
        boost::filesystem::path(shareDirectory()) / "model/PA10/PA10.body");
            
    BodyLoader loader;
    loader.setMessageSink(cout);
    loader.setShapeLoadingEnabled(false);
    body = loader.load(modelfile);
            
    if(!body){
        cout << modelfile << " cannot be loaded." << endl;
        return RTC::RTC_ERROR;
    }

    numJoints = body->numJoints();
    leftHand_id  = body->link("HAND_L")->jointId();
    rightHand_id = body->link("HAND_R")->jointId();

    return RTC::RTC_OK;
}


RTC::ReturnCode_t PA10PickupControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    time = 0.0;
    qref.resize(numJoints);
    qref_old.resize(numJoints);
    qold.resize(numJoints);

    wrist = body->link("J7");
    Link* base = body->rootLink();
    baseToWrist = getCustomJointPath(body, base, wrist);
    base->p().setZero();
    base->R().setIdentity();
        
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }
    for(int i=0; i < numJoints; ++i){
        double q = m_angle.data[i];
        qold[i] = q;
        body->joint(i)->q() = q;
    }
    qref = qold;
    qref_old = qold;
    baseToWrist->calcForwardKinematics();

    VectorXd p0(6);
    p0.head<3>() = wrist->p();
    p0.tail<3>() = rpyFromRot(wrist->attitude());

    VectorXd p1(6);
    p1.head<3>() = Vector3(0.9, 0.0, 0.25);
    p1.tail<3>() = toRadianVector3(180.0, 0.0, 0.0);

    wristInterpolator.clear();
    wristInterpolator.appendSample(0.0, p0);
    wristInterpolator.appendSample(1.0, p1);
    p1.z() = 0.2;
    wristInterpolator.appendSample(1.2, p1);
    wristInterpolator.update();

    phase = 0;
    dq_hand = 0.0;
	
    m_torque_out.data.length(numJoints);
    for(int i=0; i < numJoints; ++i){
        m_torque_out.data[i] = 0.0;
    }

    return RTC::RTC_OK;
}


RTC::ReturnCode_t PA10PickupControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t PA10PickupControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    VectorXd p(6);
    if(phase <= 3){
        p = wristInterpolator.interpolate(time);
        if(baseToWrist->calcInverseKinematics(
               Vector3(p.head<3>()), wrist->calcRfromAttitude(rotFromRpy(Vector3(p.tail<3>()))))){
            for(int i=0; i < baseToWrist->numJoints(); ++i){
                Link* joint = baseToWrist->joint(i);
                qref[joint->jointId()] = joint->q();
            }
        }            
    }

    if(phase == 0){
        if(time > wristInterpolator.domainUpper()){
            phase = 1;
        }

    } else if(phase == 1){
        if(fabs(m_torque_out.data[leftHand_id]) < 40.0 ||
           fabs(m_torque_out.data[rightHand_id]) < 40.0){ // not holded ?
            dq_hand = std::min(dq_hand + 0.00001, 0.0005);
            qref[rightHand_id] -= radian(dq_hand);
            qref[leftHand_id]  += radian(dq_hand);

        } else {
            VectorXd p2(6);
            p2.head<3>() = Vector3(0.0, 0.5, 1.0);
            p2.tail<3>() = toRadianVector3(180.0, -45, 90.0);

            VectorXd p3(6);
            p3.head<3>() = Vector3(0.0, 0.7, 0.52);
            p3.tail<3>() = toRadianVector3(180.0, 0, 90.0);

            wristInterpolator.clear();
            wristInterpolator.appendSample(time, p);
            wristInterpolator.appendSample(time + 1.0, p2);
            wristInterpolator.appendSample(time + 1.5, p3);
            wristInterpolator.appendSample(time + 1.7, p3);
            wristInterpolator.update();
            phase = 2;
        }
    } else if(phase == 2){
        if(time > wristInterpolator.domainUpper()){
            phase = 3;
            dq_hand = 0.0;
        }
    } else if(phase == 3){
        if(qref[rightHand_id] < 0.028 || qref[leftHand_id] > -0.028){
            dq_hand = std::min(dq_hand + 0.00001, 0.002);
            qref[rightHand_id] += radian(dq_hand);
            qref[leftHand_id]  -= radian(dq_hand);
        } else {
            jointInterpolator.clear();
            jointInterpolator.appendSample(time, qref);
            VectorXd qf = VectorXd::Zero(qref.size());
            qf[rightHand_id] = qref[rightHand_id];
            qf[leftHand_id]  = qref[leftHand_id];
            jointInterpolator.appendSample(time + 1.0, qf);
            jointInterpolator.update();
            phase = 4;
        }
    } else if(phase == 4){
        qref = jointInterpolator.interpolate(time);
    }

    for(int i=0; i < numJoints; ++i){
        double q = m_angle.data[i];
        double dq = (q - qold[i]) / TIMESTEP;
        double dq_ref = (qref[i] - qref_old[i]) / TIMESTEP;
        m_torque_out.data[i] = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
        qold[i] = q;
    }

    qref_old = qref;
    time += TIMESTEP;
    
    m_torqueOut.write();
  
    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void PA10PickupControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<PA10PickupControllerRTC>,
                                 RTC::Delete<PA10PickupControllerRTC>);
    }
};
