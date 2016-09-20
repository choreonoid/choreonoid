/*!
  @brief A component to save the constraint force data to a file.
*/

#include "ConstraintLogRTC.h"

#define FILENAME "SR1wristConstraintForce.log"

// Module specification
static const char* logFile_spec[] =
{
    "implementation_id", "ConstraintLogRTC",
    "type_name",         "ConstraintLogRTC",
    "description",       "save the constraint force data to a file.",
    "version",           "1.0.0",
    "vendor",            "AIST",
    "category",          "Human input",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
};


ConstraintLogRTC::ConstraintLogRTC(RTC::Manager* manager)
    : RTC::DataFlowComponentBase(manager),
      m_larmCFIn("larm_cf", m_larmCF),
      m_rarmCFIn("rarm_cf", m_rarmCF)
{

}


ConstraintLogRTC::~ConstraintLogRTC()
{

}


RTC::ReturnCode_t ConstraintLogRTC::onInitialize()
{
    addInPort("larm_cf", m_larmCFIn);
    addInPort("rarm_cf", m_rarmCFIn);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t ConstraintLogRTC::onActivated(RTC::UniqueId ec_id)
{
    outFile.open(FILENAME, std::ios::out);

    return RTC::RTC_OK;
}


RTC::ReturnCode_t ConstraintLogRTC::onDeactivated(RTC::UniqueId ec_id)
{
    outFile.close();

    return RTC::RTC_OK;
}



RTC::ReturnCode_t ConstraintLogRTC::onExecute(RTC::UniqueId ec_id)
{
    if(m_larmCFIn.isNew()){
        m_larmCFIn.read();
        m_rarmCFIn.read();

        outFile << m_larmCF.tm.sec + m_larmCF.tm.nsec / 1e9 << " :" << std::endl;

        outFile << "LARM_WRIST :" << std::endl;
        for(int i=0; i<m_larmCF.data.length(); i++){
            outFile << m_larmCF.data[i] << " ";
            if((i+1)%6==0)
                outFile << std::endl;
        }

        outFile << "RARM_WRIST :" << std::endl;
        for(int i=0; i<m_rarmCF.data.length(); i++){
            outFile << m_rarmCF.data[i] << " ";
            if((i+1)%6==0)
                outFile << std::endl;
        }

        outFile << std::endl;
    }
    
    return RTC::RTC_OK;
}


extern "C"
{
    void ConstraintLogRTCInit(RTC::Manager* manager)
    {
        coil::Properties profile(logFile_spec);
        manager->registerFactory(profile,
                                 RTC::Create<ConstraintLogRTC>,
                                 RTC::Delete<ConstraintLogRTC>);
    }
};
