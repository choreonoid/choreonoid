/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include "TCSimulatorItem.h"
#include <cnoid/SubSimulatorItem>
#include <cnoid/Body>
#include <map>

#ifndef NIC_MAX
#define NIC_MAX (50)
#endif

#ifndef NIC_DIR
#define NIC_DIR "/sys/class/net"
#endif

#ifndef REF_MIN
#define REF_MIN (-1000000.0)
#endif

#ifndef REF_MAX
#define REF_MAX (1000000.0)
#endif

#ifndef TSTEP_MIN
#define TSTEP_MIN (0.001)
#endif

#ifndef TSTEP_MAX
#define TSTEP_MAX (3600.0)
#endif

#ifndef TSTEP_DEFAULT
#define TSTEP_DEFAULT (1.0)
#endif

#ifndef TC_CONF_FILE
#define TC_CONF_FILE "/usr/local/share/cnoid-conf/tc.conf"
#endif

class DynamicTCSimulatorItem : public cnoid::SubSimulatorItem
{
public:

    DynamicTCSimulatorItem();

    DynamicTCSimulatorItem(const DynamicTCSimulatorItem& org);

    ~DynamicTCSimulatorItem();

    static void initializeClass(cnoid::ExtensionManager* extMgr);

    virtual bool initializeSimulation(cnoid::SimulatorItem* simulatorItem);

    virtual void finalizeSimulation();

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

    int listEther(const char *base_path);
    double calcDistance(const cnoid::Vector3& p1,const cnoid::Vector3& p2);
    void bridgeTC(const int& ethIdxNo,const double& distance);
    bool checkTcsInstance();
    void checkBodyItem();
    bool findNIC(const std::string &nic);
    bool chkEnables();

    int _preFuncId, _midFuncId, _postFuncId;

    cnoid::Selection _communicationPort;
    bool _dTCFlagC[NIC_MAX];
    bool _dTCFlagP[NIC_MAX];
    cnoid::Vector3 _referencePointC[NIC_MAX];
    cnoid::Vector3 _referencePointP[NIC_MAX];
    std::string _targetBody[NIC_MAX];
    double _timeStep[NIC_MAX];

    std::string _ethName[NIC_MAX];
    std::vector<std::string> _allNIC;
    int _staticEthIndex[NIC_MAX];
    std::string _staticEthName[NIC_MAX];
    cnoid::Body *_body[NIC_MAX];
    double _nextTime[NIC_MAX];
    bool _nicDirChk=false;
    std::string _config;
    std::map<std::string,std::string> _pair;

    int _idxCur = 0;

    int _idxNew = -1;

    bool _portChanged = false;

    TCSimulatorItem *_tcs = nullptr;

    cnoid::SimulatorItem* _curSimItem = nullptr;

    bool _setSignal = false;
};

typedef cnoid::ref_ptr<DynamicTCSimulatorItem> DynamicTCSimulatorItemPtr;
