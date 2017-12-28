/**
   @author Shizuko Hattori
   @author Shin'ichiro Nakaoka
*/

#include "SR1WalkControllerRTC.h"
#include <cnoid/BodyMotion>
#include <cnoid/ExecutablePath>
#include <cnoid/FileUtil>

using namespace std;
using namespace cnoid;

namespace {

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
    "implementation_id", "SR1WalkControllerRTC",
    "type_name",         "SR1WalkControllerRTC",
    "description",       "OpenRTM_SR1 Walk Controller component",
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


SR1WalkControllerRTC::SR1WalkControllerRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_angleIn("q", m_angle),
      m_torqueOut("u", m_torque)
{

}

SR1WalkControllerRTC::~SR1WalkControllerRTC()
{

}


RTC::ReturnCode_t SR1WalkControllerRTC::onInitialize()
{
    // Set InPort buffers
    addInPort("q", m_angleIn);

    // Set OutPort buffer
    addOutPort("u", m_torqueOut);

    return RTC::RTC_OK;
}

RTC::ReturnCode_t SR1WalkControllerRTC::onActivated(RTC::UniqueId ec_id)
{
    if(!qseq){
        string filename = getNativePathString(
            boost::filesystem::path(shareDirectory())
            / "motion" / "SR1" / "SR1WalkPattern1.seq");

        BodyMotion motion;
        if(!motion.loadStandardYAMLformat(filename)){
            cout << motion.seqMessage() << endl;
            return RTC::RTC_ERROR;
        }
        qseq = motion.jointPosSeq();
        if(qseq->numFrames() == 0){
            cout << "Empty motion data." << endl;
            return RTC::RTC_ERROR;
        }
        q0.resize(qseq->numParts());
        timeStep_ = qseq->getTimeStep();
    }

    m_torque.data.length(qseq->numParts());

    if(m_angleIn.isNew()){
        m_angleIn.read();
    }
    for(int i=0; i < qseq->numParts(); ++i){
        q0[i] = m_angle.data[i];
    }
    oldFrame = qseq->frame(0);
    currentFrame = 0;

    return RTC::RTC_OK;
}


RTC::ReturnCode_t SR1WalkControllerRTC::onDeactivated(RTC::UniqueId ec_id)
{
    return RTC::RTC_OK;
}

RTC::ReturnCode_t SR1WalkControllerRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_angleIn.isNew()){
        m_angleIn.read();
    }

    if(currentFrame < qseq->numFrames()){

        MultiValueSeq::Frame frame = qseq->frame(currentFrame++);

        for(int i=0; i < frame.size(); i++){
            double q_ref = frame[i];
            double q = m_angle.data[i];
            double dq_ref = (q_ref - oldFrame[i]) / timeStep_;
            double dq = (q - q0[i]) / timeStep_;
            m_torque.data[i] = (q_ref - q) * pgain[i] + (dq_ref - dq) * dgain[i];
            q0[i] = q;
        }
        oldFrame = frame;
    }
    m_torqueOut.write();

    return RTC::RTC_OK;
}

extern "C"
{
    DLL_EXPORT void SR1WalkControllerRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(samplepd_spec);
        manager->registerFactory(profile,
                                 RTC::Create<SR1WalkControllerRTC>,
                                 RTC::Delete<SR1WalkControllerRTC>);
    }
};
