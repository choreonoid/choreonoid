/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "SR1LiftupControllerRTC.h"
#include <cnoid/ExecutablePath>
#include <cnoid/BodyLoader>
#include <cnoid/Link>
#include <cnoid/EigenUtil>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

namespace {

const double TIMESTEP = 0.002;

VectorXd& convertToRadian(VectorXd& q){
    for(int i=0; i < q.size(); ++i){
        q[i] = radian(q[i]);
    }
    return q;
}

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0 };

const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

const char* samplepd_spec[] =
{
    "implementation_id", "SR1LiftupControllerRTC",
    "type_name",         "SR1LiftupControllerRTC",
    "description",       "OpenRTM_SR1 Liftup Controller component",
    "version",           "0.1",
    "vendor",            "AIST",
    "category",          "Generic",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};
}


SR1LiftupControllerRTC::SR1LiftupControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_torqueOut("u_out", m_torque_out)
{

}


SR1LiftupControllerRTC::~SR1LiftupControllerRTC()
{

}


RTC::ReturnCode_t SR1LiftupControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);
  
    // Set OutPort buffer
    addOutPort("u_out", m_torqueOut);

    string modelfile = getNativePathString(
        boost::filesystem::path(shareDirectory()) / "model/SR1/SR1.body");
            
    BodyLoader loader;
    loader.setMessageSink(cout);
    loader.setShapeLoadingEnabled(false);
    body = loader.load(modelfile);
            
    if(!body){
        cout << modelfile << " cannot be loaded." << endl;
        return RTC::RTC_ERROR;
    }

    numJoints = body->numJoints();
    rightWrist_id = body->link("RARM_WRIST_R")->jointId();
    leftWrist_id = body->link("LARM_WRIST_R")->jointId();

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SR1LiftupControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    time = 0.0;
    throwTime = std::numeric_limits<double>::max();

    qref_old.resize(numJoints);
    qold.resize(numJoints);

    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    VectorXd q0(numJoints);
    for(size_t i=0; i < numJoints; ++i){
        qold[i] = m_angle.data[i];
        q0[i] = m_angle.data[i];
    }

    VectorXd q1(numJoints);
    q1 <<
        0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
        10.0,   0.0, 0.0, -90.0,   0.0, 0.0, 0.0,
        0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
        10.0,   0.0, 0.0, -90.0,   0.0, 0.0, 0.0,
        0.0,   0.0, 0.0;

    VectorXd q2(numJoints);
    q2 <<
        0.0, -80.0, 0.0, 148.0, -70.0, 0.0,
        -47.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
        0.0, -80.0, 0.0, 148.0, -70.0, 0.0,
        -47.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
        50.0,   0.0, 0.0;
            
    interpolator.clear();
    interpolator.appendSample(0.0, q0);
    interpolator.appendSample(1.5, convertToRadian(q1));
    interpolator.appendSample(4.5, convertToRadian(q2));
    interpolator.update();

    qref_old = interpolator.interpolate(0.0);

    phase = 0;
    dq_wrist = 0.0;

    m_torque_out.data.length(numJoints);
    for(size_t i=0; i < numJoints; ++i){
        m_torque_out.data[i] = 0.0;
    }

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SR1LiftupControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}


RTC::ReturnCode_t SR1LiftupControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    if(phase == 0){
        qref = interpolator.interpolate(time);
        if(time > interpolator.domainUpper()){
            phase = 1;
        }

    } else if(phase == 1){
        // holding phase
        qref = qref_old;

        if(fabs(m_torque_out.data[rightWrist_id]) < 50.0 || fabs(m_torque_out.data[leftWrist_id]) < 50.0){ // not holded ?
            dq_wrist = std::min(dq_wrist + 0.001, 0.1);
            qref[rightWrist_id] += radian(dq_wrist);
            qref[leftWrist_id]  -= radian(dq_wrist);
            
        } else {
            // transit to getting up phase
            VectorXd q3(numJoints);
            q3 <<
                0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                0.0,   0.0, 0.0;
            convertToRadian(q3);
            q3[rightWrist_id] = qref[rightWrist_id];
            q3[leftWrist_id]  = qref[leftWrist_id];
            
            interpolator.clear();
            interpolator.appendSample(time, qref);
            interpolator.appendSample(time + 2.5, q3);
            interpolator.appendSample(time + 4.0, q3);
            interpolator.update();
            qref = interpolator.interpolate(time);
            phase = 2;
        }
        
    } else if(phase == 2){
        qref = interpolator.interpolate(time);
        
        if(time > interpolator.domainUpper()){
            // transit to throwing phase
            VectorXd q4(numJoints);
            q4 <<
                0.0, -40.0, 0.0,  80.0, -40.0, 0.0,
                -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                0.0, -40.0, 0.0,  80.0, -40.0, 0.0,
                -50.0,   0.0, 0.0, -60.0,   0.0, 0.0, 0.0,
                10.0,   0.0, 0.0;
            convertToRadian(q4);
            q4[rightWrist_id] = qref[rightWrist_id];
            q4[leftWrist_id]  = qref[leftWrist_id];

            VectorXd q5(numJoints);
            q5 <<
                0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                0.0, -15.0, 0.0,  45.0, -30.0, 0.0,
                -60.0,   0.0, 0.0, -50.0,   0.0, 0.0, 0.0,
                0.0,   0.0, 0.0;
            convertToRadian(q5);
            q5[rightWrist_id] = qref[rightWrist_id];
            q5[leftWrist_id]  = qref[leftWrist_id];

            interpolator.clear();
            interpolator.appendSample(time, qref);
            interpolator.setEndPoint(interpolator.appendSample(time + 1.0, q4));
            interpolator.appendSample(time + 1.3, q5);
            throwTime = time + 1.15;
            interpolator.update();

            qref = interpolator.interpolate(time);
            phase = 3;
        }

    } else if (phase == 3){
        qref = interpolator.interpolate(time);

    } else if (phase == 4){
        qref[rightWrist_id] = 0.0; 
        qref[leftWrist_id]  = 0.0; 
    }

    for(size_t i=0; i < numJoints; ++i){
        double q = m_angle.data[i];
        double dq = (q - qold[i]) / TIMESTEP;
        double dq_ref = (qref[i] - qref_old[i]) / TIMESTEP;
        m_torque_out.data[i] = (qref[i] - q) * pgain[i] + (dq_ref - dq) * dgain[i];
        qold[i] = q;
    }

    if(phase == 3){
        if(time > throwTime){
            if(time < interpolator.domainUpper() + 0.1){
                m_torque_out.data[rightWrist_id] = 0.0;
                m_torque_out.data[leftWrist_id] = 0.0;
            } else {
                phase = 4;
            }
        }
    }
        
    qref_old = qref;

    time += TIMESTEP;
    
    m_torqueOut.write();
  
    return RTC::RTC_OK;
}


extern "C"
{
    DLL_EXPORT void SR1LiftupControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SR1LiftupControllerRTC>,
                                 RTC::Delete<SR1LiftupControllerRTC>);
    }
};
