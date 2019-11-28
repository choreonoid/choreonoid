/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <cnoid/SubSimulatorItem>
#include <cnoid/Selection>
#include <map>
#include <vector>
#include "exportdecl.h"

#ifndef NIC_MAX
#define NIC_MAX (50)
#endif

#ifndef NIC_DIR
#define NIC_DIR "/sys/class/net"
#endif

#ifndef ZERO
#define ZERO (0)
#endif

#ifndef DELAY_MAX
#define DELAY_MAX (100000)
#endif

#ifndef BAND_MAX
#define BAND_MAX (11000000)
#endif

#ifndef LOSS_MAX
#define LOSS_MAX (100.0)
#endif

#ifndef TC_CONF_FILE
#define TC_CONF_FILE "/usr/local/share/cnoid-conf/tc.conf"
#endif

#ifndef IP_SEPARATOR
#define IP_SEPARATOR ','
#endif

class CNOID_EXPORT TCSimulatorItem : public cnoid::SubSimulatorItem
{
public:

    TCSimulatorItem();

    TCSimulatorItem(const TCSimulatorItem& org);

    ~TCSimulatorItem();

    static void initializeClass(cnoid::ExtensionManager* extMgr);

    virtual bool initializeSimulation(cnoid::SimulatorItem* simulatorItem);

    virtual void finalizeSimulation();


    bool isEnableTrafficControl() { return _enableTrafficControl; }
    int getEthIndexNo(const std::string &ethName);
    void bridgeTC(const int& ethIdxNo,const double& distance,const double& upDelay,const double& upRate,const double& upLoss,const double& dnDelay,const double& dnRate,const double& dnLoss);
    void bridgeInit();

protected:
    virtual cnoid::Item* doDuplicate() const;
    virtual void doPutProperties(cnoid::PutPropertyFunction& putProperty);
    virtual bool store(cnoid::Archive& archive);
    virtual bool restore(const cnoid::Archive& archive);

private:
    void onPreDynamicFunction();
    void onMidDynamicFunction();
    void onPostDynamicFunction();
    void onPaused();
    void onResumed();

    void sysCall(const std::string& cmdStr);
    void sysCall2(const std::string& cmdStr,const std::string& nic,const char *num);
    void sysCall3(const std::string& cmdStr,const std::string& nic,const char *num); //!< 外部コマンドを呼び出す(preCheckあり)
    int listEther(const char *base_path);
    bool chkIPMask(const std::string& ip);
    bool chkDigits(const char *p);
    void setVirNIC();
    void doTC();
    void doTC(const int& ethIdxNo);
    void doTC2com(const int& ethIdxNo,const int *p1,const int *p2,const double *p3,const int *p4,const int *p5,const double *p6);
    void doTC2S(const int& ethIdxNo);
    void doTC2D(const int& ethIdxNo);
    std::vector<std::string> split(const std::string &str,const char& delim);
    std::string rangeStr(const int &min,const int &max);
    void initTC();
    void resetTC();
    bool findNIC(const std::string &nic);

    int _preFuncId, _midFuncId, _postFuncId;

    bool _enableTrafficControl;
    cnoid::Selection _communicationPort;

    int    _OutboundDelay[NIC_MAX];
    int    _OutboundBandWidth[NIC_MAX];
    double _OutboundLoss[NIC_MAX];
    int    _InboundDelay[NIC_MAX];
    int    _InboundBandWidth[NIC_MAX];
    double _InboundLoss[NIC_MAX];

    int    _OutboundDelayD[NIC_MAX];
    int    _OutboundBandWidthD[NIC_MAX];
    double _OutboundLossD[NIC_MAX];
    int    _InboundDelayD[NIC_MAX];
    int    _InboundBandWidthD[NIC_MAX];
    double _InboundLossD[NIC_MAX];

    std::string _ipAddressC[NIC_MAX];
    std::string _ipAddressP[NIC_MAX];
    std::string _ipAddressT[NIC_MAX];

    std::string _ethName[NIC_MAX];
    std::string _virName[NIC_MAX];
    std::vector<std::string> _allNIC;

    std::map<std::string,std::string> _pair;
    bool _nicDirChk=false;
    std::string _config;

    int _idxCur = 0;
    int _idxNew = -1;

    bool _portChanged=false;

    cnoid::SimulatorItem* _curSimItem;

    bool _setSignal = false;

    bool _initTC = false;

    bool _monitorDynamicTC = false;
};

typedef cnoid::ref_ptr<TCSimulatorItem> TCSimulatorItemPtr;
