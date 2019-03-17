/**
   @author Japan Atomic Energy Agency
*/

#include "TCSimulatorItem.h"
#include "TrafficControlShare.h"
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/SimulatorItem>
#include <cnoid/Archive>
#include <dirent.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

void
TCSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<TCSimulatorItem>("TCSimulatorItem");
    im.addCreationPanel<TCSimulatorItem>();
}

TCSimulatorItem::TCSimulatorItem()
{
    _curSimItem = nullptr;
    _preFuncId = _midFuncId = _postFuncId = -1;
    _enableTrafficControl=true;

    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsRunning(false);

    int cnt=0;

    _config=std::string(TC_CONF_FILE);
    char file[_config.size()+1];
    strcpy(file,_config.c_str());

    FILE *fp=fopen(file,"r");

    _pair.clear();

    if(fp!=NULL) {
        char line[1024];
        vector<string> list;
        int lcnt=0;
        while(fgets(line,sizeof line,fp)!=NULL) {
            int l=strlen(line)-1;
            if(l>=0) {
                line[l]='\0';
            }
            char *p=strstr(line,",");
            if(p==NULL||line[0]=='#'||lcnt==0) {

            }
            else {
                if(_nicDirChk==false) {
                    cnt=listEther(NIC_DIR);
                    _nicDirChk=true;
                }

                char *p1=line;
                *p='\0';
                char *p2=p+1;

                string n1=std::string(p1);
                string n2=std::string(p2);

                bool rc1=findNIC(n1);
                bool rc2=findNIC(n2);

                bool rc3=false;
                bool rc4=false;

                if(std::find(list.begin(),list.end(),n1)!=list.end()) {

                }
                else {
                    rc3=true;
                    list.push_back(n1);
                }

                if(std::find(list.begin(),list.end(),n2)!=list.end()) {

                }
                else {
                    rc4=true;
                    list.push_back(n2);
                }

                if(rc1&&rc2&&rc3&&rc4) {
                    _pair[n1] = n2;
                } else {
                    if(rc1==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem "+n1+" does not exist in this computer. Please check configure file="+_config);
                    }
                    if(rc2==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem "+n2+" does not exist in this computer. Please check configure file="+_config);
                    }
                    if(rc3==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem "+n1+" is already defined(duplicated). Please check configure file="+_config);
                    }
                    if(rc4==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem "+n2+" is already defined(duplicated). Please check configure file="+_config);
                    }
                }
            }
            lcnt++;
        }
        fclose(fp);
    }
    else {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem Configure file="+_config+" does not exist.");
    }

    cnt = 0;
    for(auto itr = _pair.begin(); itr!=_pair.end();++itr) {
        string eth=itr->first;
        if(cnt<NIC_MAX) {
            _ethName[cnt]=eth;
            _communicationPort.setSymbol(cnt,eth);
            cnt++;
        }
        else {
            MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem NIC_MAX=="+std::to_string(NIC_MAX)+", so "+eth+" is not available. Please redefine NIC_MAX value more larger.");
        }
    }

    if(cnt>0) {
        _communicationPort.resize(cnt);
    }
    else {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::TCSimulatorItem Configure file="+_config+" is invalid. Effective Port is nothing.");
        string eth="No valid port exists.";
        _ethName[cnt]=eth;
        _communicationPort.setSymbol(0,eth);
        _communicationPort.resize(1);
    }

    for(int i=0;i<NIC_MAX;i++) {
        _OutboundDelay[i]=0;
        _OutboundBandWidth[i]=0;
        _OutboundLoss[i]=0;
        _InboundDelay[i]=0;
        _InboundBandWidth[i]=0;
        _InboundLoss[i]=0;

        _OutboundDelayD[i]=-1;
        _OutboundBandWidthD[i]=-1;
        _OutboundLossD[i]=-1.0;
        _InboundDelayD[i]=-1;
        _InboundBandWidthD[i]=-1;
        _InboundLossD[i]=-1.0;

        _ipAddressC[i]="";
        _ipAddressP[i]="";
        _ipAddressT[i]="";
    }

    if(_initTC==false) {
        initTC();
        _initTC=true;
    }
}

TCSimulatorItem::TCSimulatorItem(const TCSimulatorItem& org) : SubSimulatorItem(org)
{
    _enableTrafficControl=org._enableTrafficControl;
    _communicationPort=org._communicationPort;

    for(int i=0;i<NIC_MAX;i++) {
        _ethName[i] = org._ethName[i];

        _OutboundDelay[i]=org._OutboundDelay[i];
        _OutboundBandWidth[i]=org._OutboundBandWidth[i];
        _OutboundLoss[i]=org._OutboundLoss[i];
        _InboundDelay[i]=org._InboundDelay[i];
        _InboundBandWidth[i]=org._InboundBandWidth[i];
        _InboundLoss[i]=org._InboundLoss[i];

        _OutboundDelayD[i]=org._OutboundDelayD[i];
        _OutboundBandWidthD[i]=org._OutboundBandWidthD[i];
        _OutboundLossD[i]=org._OutboundLossD[i];
        _InboundDelayD[i]=org._InboundDelayD[i];
        _InboundBandWidthD[i]=org._InboundBandWidthD[i];
        _InboundLossD[i]=org._InboundLossD[i];

        _ipAddressC[i]=org._ipAddressC[i];
        _ipAddressP[i]=org._ipAddressP[i];
        _ipAddressT[i]=org._ipAddressT[i];
    }
    _allNIC=org._allNIC;
    _pair=org._pair;
}

TCSimulatorItem::~TCSimulatorItem()
{

}

Item*
TCSimulatorItem::doDuplicate() const
{
    return new TCSimulatorItem(*this);
}

bool
TCSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    setVirNIC();

    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsInstance(this);

    _curSimItem = simulatorItem;

    if(_setSignal==false) {
        simulatorItem->sigSimulationPaused().connect(std::bind(&TCSimulatorItem::onPaused, this));
        simulatorItem->sigSimulationResumed().connect(std::bind(&TCSimulatorItem::onResumed, this));
        _setSignal = true;
    }

    for(int i=0;i<NIC_MAX;i++) {
        _OutboundDelayD[i]=0;
        _OutboundBandWidthD[i]=0;
        _OutboundLossD[i]=0;
        _InboundDelayD[i]=0;
        _InboundBandWidthD[i]=0;
        _InboundLossD[i]=0;
    }

    for(int i=0;i<NIC_MAX;i++) {
        _ipAddressT[i] = "xyz";
    }

    _share->setTcsRunning(true);

    doTC();

    return true;
}

void
TCSimulatorItem::resetTC() {
    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsRunning(true);

    int OutboundDelay[NIC_MAX];
    int OutboundBandWidth[NIC_MAX];
    double OutboundLoss[NIC_MAX];
    int InboundDelay[NIC_MAX];
    int InboundBandWidth[NIC_MAX];
    double InboundLoss[NIC_MAX];
    std::string ipAddressC[NIC_MAX];
    std::string ipAddressT[NIC_MAX];
    bool enableTrafficControl;

    enableTrafficControl=_enableTrafficControl;
    for(int i=0;i<NIC_MAX;i++) {
        OutboundDelay[i]=_OutboundDelay[i];
        OutboundBandWidth[i]=_OutboundBandWidth[i];
        OutboundLoss[i]=_OutboundLoss[i];
        InboundDelay[i]=_InboundDelay[i];
        InboundBandWidth[i]=_InboundBandWidth[i];
        InboundLoss[i]=_InboundLoss[i];
        ipAddressC[i]=_ipAddressC[i];
        ipAddressT[i]=_ipAddressT[i];
    }

    _enableTrafficControl=true;
    for(int i=0;i<NIC_MAX;i++) {
        _OutboundDelay[i]=0;
        _OutboundBandWidth[i]=0;
        _OutboundLoss[i]=0;
        _InboundDelay[i]=0;
        _InboundBandWidth[i]=0;
        _InboundLoss[i]=0;
        _ipAddressC[i]="";
        _ipAddressT[i]="xyz";
    }

    doTC();

    _enableTrafficControl=enableTrafficControl;
    for(int i=0;i<NIC_MAX;i++) {
        _OutboundDelay[i]=OutboundDelay[i];
        _OutboundBandWidth[i]=OutboundBandWidth[i];
        _OutboundLoss[i]=OutboundLoss[i];
        _InboundDelay[i]=InboundDelay[i];
        _InboundBandWidth[i]=InboundBandWidth[i];
        _InboundLoss[i]=InboundLoss[i];
        _ipAddressC[i]=ipAddressC[i];
        _ipAddressT[i]=ipAddressT[i];
    }
}

void
TCSimulatorItem::finalizeSimulation()
{
    resetTC();

    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsRunning(false);

    _curSimItem = nullptr;

    _share->setTcsInstance(nullptr);
}

void
TCSimulatorItem::onPaused() {
    resetTC();

    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsRunning(false);
}

void
TCSimulatorItem::onResumed() {

    TrafficControlShare* _share = TrafficControlShare::instance();
    _share->setTcsRunning(true);

    for(int i=0;i<NIC_MAX;i++) {
        _ipAddressT[i] = "xyz";
    }

    doTC();
}

void
TCSimulatorItem::onPreDynamicFunction()
{

}

void
TCSimulatorItem::onMidDynamicFunction()
{

}

void
TCSimulatorItem::onPostDynamicFunction()
{

}

void
TCSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);

    putProperty(_("EnableTrafficControl"),_enableTrafficControl, changeProperty(_enableTrafficControl));

    putProperty(_("Port"), _communicationPort,
                [&](int index){ _portChanged = true; _idxNew = index; return _communicationPort.selectIndex(index);});

    if(_portChanged) {

        putProperty.min(ZERO);
        putProperty.max(DELAY_MAX);
        putProperty(_("OutboundDelay[ms]"),_OutboundDelay[_idxNew], changeProperty(_OutboundDelay[_idxNew]));

        putProperty.min(ZERO);
        putProperty.max(BAND_MAX);
        putProperty(_("OutboundBandWidth[kbit/s]"), _OutboundBandWidth[_idxNew], changeProperty(_OutboundBandWidth[_idxNew]));

        putProperty.decimals(3).min(ZERO);
        putProperty.decimals(3).max(LOSS_MAX);
        putProperty(_("OutboundLoss[%]"), _OutboundLoss[_idxNew], changeProperty(_OutboundLoss[_idxNew]));

        putProperty.min(ZERO);
        putProperty.max(DELAY_MAX);
        putProperty(_("InboundDelay[ms]"),_InboundDelay[_idxNew], changeProperty(_InboundDelay[_idxNew]));

        putProperty.min(ZERO);
        putProperty.max(BAND_MAX);
        putProperty(_("InboundBandWidth[kbit/s]"), _InboundBandWidth[_idxNew], changeProperty(_InboundBandWidth[_idxNew]));

        putProperty.decimals(3).min(ZERO);
        putProperty.decimals(3).max(LOSS_MAX);
        putProperty(_("InboundLoss[%]"), _InboundLoss[_idxNew], changeProperty(_InboundLoss[_idxNew]));

        putProperty(_("IP Address"), _ipAddressC[_idxNew], changeProperty(_ipAddressC[_idxNew]));

        _portChanged = false;
        _idxCur = _idxNew;
    }
    else {
        putProperty.min(ZERO);
        putProperty.max(DELAY_MAX);
        putProperty(_("OutboundDelay[ms]"),_OutboundDelay[_idxCur], changeProperty(_OutboundDelay[_idxCur]));

        putProperty.min(ZERO);
        putProperty.max(BAND_MAX);
        putProperty(_("OutboundBandWidth[kbit/s]"), _OutboundBandWidth[_idxCur], changeProperty(_OutboundBandWidth[_idxCur]));

        putProperty.decimals(3).min(0.0);
        putProperty.decimals(3).max(LOSS_MAX);
        putProperty(_("OutboundLoss[%]"), _OutboundLoss[_idxCur], changeProperty(_OutboundLoss[_idxCur]));

        putProperty.min(ZERO);
        putProperty.max(DELAY_MAX);
        putProperty(_("InboundDelay[ms]"),_InboundDelay[_idxCur], changeProperty(_InboundDelay[_idxCur]));

        putProperty.min(ZERO);
        putProperty.max(BAND_MAX);
        putProperty(_("InboundBandWidth[kbit/s]"), _InboundBandWidth[_idxCur], changeProperty(_InboundBandWidth[_idxCur]));

        putProperty.decimals(3).min(0.0);
        putProperty.decimals(3).max(LOSS_MAX);
        putProperty(_("InboundLoss[%]"), _InboundLoss[_idxCur], changeProperty(_InboundLoss[_idxCur]));

        if(_ipAddressC[_idxCur].compare("")==0) {
            putProperty(_("IP Address"), _ipAddressC[_idxCur], changeProperty(_ipAddressC[_idxCur]));
        }
        else {
            bool rc = chkIPMask(_ipAddressC[_idxCur]);

            if(rc==false) {
                MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doPutProperties IP Address/Mask = \""+_ipAddressC[_idxCur]+"\" is an invalid string.");
                MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doPutProperties Please specify empty string or 192.168.0.1/24 format string.");

                _ipAddressC[_idxCur] = _ipAddressP[_idxCur];
                putProperty(_("IP Address"), _ipAddressC[_idxCur], changeProperty(_ipAddressC[_idxCur]));
            }
            else {
                putProperty(_("IP Address"), _ipAddressC[_idxCur], changeProperty(_ipAddressC[_idxCur]));
            }
        }
    }

    _ipAddressP[_idxCur]=_ipAddressC[_idxCur];
}

bool
TCSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);

    archive.write("EnableTrafficControl",_enableTrafficControl);

    int portCount=_communicationPort.size();
    archive.write("PortCount",portCount);

    string portName="";
    for(int i=0;i<portCount;i++){
        portName="Port"+std::to_string(i);
        archive.write(portName, _communicationPort.symbol(i));
        archive.write(portName+"OutboundDelay",_OutboundDelay[i]);
        archive.write(portName+"OutboundBandWidth", _OutboundBandWidth[i]);
        archive.write(portName+"OutboundLoss", _OutboundLoss[i]);
        archive.write(portName+"InboundDelay",_InboundDelay[i]);
        archive.write(portName+"InboundBandWidth", _InboundBandWidth[i]);
        archive.write(portName+"InboundLoss", _InboundLoss[i]);
        archive.write(portName+"IPAddress", _ipAddressC[i]);
    }

    return true;
}

string
TCSimulatorItem::rangeStr(const int &min,const int &max) {
    string rc="("+std::to_string(min)+"-"+std::to_string(max)+")";
    return rc;
}

bool
TCSimulatorItem::findNIC(const std::string &nic) {
    bool find=false;

    for(int j=0;j<_allNIC.size();j++) {
        if(_allNIC[j].compare(nic)==0) {
            find=true;
            break;
        }
    }

    return find;
}


bool
TCSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);

    archive.read("EnableTrafficControl",_enableTrafficControl);

    int portCount = -1;
    archive.read("PortCount",portCount);

    if(portCount>0) {
        _communicationPort.select(0);

        string portName="";
        string tmp;
        int cnt=0;
        for(int i=0;i<portCount;i++){
            portName="Port"+std::to_string(i);
            archive.read(portName, tmp);
            if(cnt<NIC_MAX) {
                bool rc1=findNIC(tmp);
                bool rc2=_pair.count(tmp)==1;
                if(rc1==false) {
                    MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+" does not exist in this computer.");
                }
                else if(rc2==false) {
                    MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+" does not defined in configure file="+_config);
                }

                if(rc1&&rc2) {
                    _communicationPort.setSymbol(cnt,tmp);
                    _ethName[cnt]=tmp;
                    archive.read(portName+"OutboundDelay",_OutboundDelay[cnt]);
                    archive.read(portName+"OutboundBandWidth", _OutboundBandWidth[cnt]);
                    archive.read(portName+"OutboundLoss", _OutboundLoss[cnt]);
                    archive.read(portName+"InboundDelay",_InboundDelay[cnt]);
                    archive.read(portName+"InboundBandWidth", _InboundBandWidth[cnt]);
                    archive.read(portName+"InboundLoss", _InboundLoss[cnt]);
                    archive.read(portName+"IPAddress", _ipAddressC[cnt]);
                    if(_OutboundDelay[cnt]<ZERO||_OutboundDelay[cnt]>DELAY_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":OutboundDelay value="+std::to_string(_OutboundDelay[cnt])+" is out of range."+rangeStr(ZERO,DELAY_MAX));
                        _OutboundDelay[cnt] = 0;
                    }
                    if(_OutboundBandWidth[cnt]<ZERO||_OutboundBandWidth[cnt]>BAND_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":OutboundBandWidth value="+std::to_string(_OutboundBandWidth[cnt])+" is out of range."+rangeStr(ZERO,BAND_MAX));
                        _OutboundBandWidth[cnt] = 0;
                    }
                    if(_OutboundLoss[cnt]<ZERO||_OutboundLoss[cnt]>LOSS_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":OutboundLoss value="+std::to_string(_OutboundLoss[cnt])+" is out of range."+rangeStr(ZERO,LOSS_MAX));
                        _OutboundLoss[cnt] = 0;
                    }
                    if(_InboundDelay[cnt]<ZERO||_InboundDelay[cnt]>DELAY_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":InboundDelay value="+std::to_string(_InboundDelay[cnt])+" is out of range."+rangeStr(ZERO,DELAY_MAX));
                        _InboundDelay[cnt] = 0;
                    }
                    if(_InboundBandWidth[cnt]<ZERO||_InboundBandWidth[cnt]>BAND_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":InboundBandWidth value="+std::to_string(_InboundBandWidth[cnt])+" is out of range."+rangeStr(ZERO,BAND_MAX));
                        _InboundBandWidth[cnt] = 0;
                    }
                    if(_InboundLoss[cnt]<ZERO||_InboundLoss[cnt]>LOSS_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":InboundLoss value="+std::to_string(_InboundLoss[cnt])+" is out of range."+rangeStr(ZERO,LOSS_MAX));
                        _InboundLoss[cnt] = 0;
                    }

                    bool rc = chkIPMask(_ipAddressC[cnt]);
                    if(rc==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":IP Address/Mask = \""+_ipAddressC[cnt]+"\" is an invalid string.");
                        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore "+tmp+":Please specify empty string or 192.168.0.1/24 format string.");
                        _ipAddressC[cnt] = "";
                    }
                    cnt++;
                }
            } else {
                MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore NIC_MAX=="+std::to_string(NIC_MAX)+", Properties of "+tmp+" can not be restored. Please redefine NIC_MAX value more larger.");
            }
        }
        if(cnt>0) {
            _communicationPort.resize(cnt);
        }
        else {
            MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore This Project file is invalid. Effective Port is nothing.");
            cnt=0;
            for(auto itr = _pair.begin(); itr!=_pair.end();++itr) {
                string eth=itr->first;
                if(cnt<NIC_MAX) {
                    _ethName[cnt]=eth;
                    _communicationPort.setSymbol(cnt,eth);
                    cnt++;
                }
                else {
                    MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore NIC_MAX=="+std::to_string(NIC_MAX)+", so "+eth+" is not available. Please redefine NIC_MAX value more larger.");
                }
            }
            if(cnt>0) {
                _communicationPort.resize(cnt);
            }
            else {
                MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::restore This Project file is invalid and Configure file="+_config+" is invalid too.");
                string eth="No valid port exists.";
                _ethName[cnt]=eth;
                _communicationPort.setSymbol(0,eth);
                _communicationPort.resize(1);
            }
        }
    }

    return true;
}

void
TCSimulatorItem::sysCall(const std::string& cmdStr) {

    int len=cmdStr.length()+5;
    char cmd[len+1];
    sprintf(cmd,"sudo %s",cmdStr.c_str());

    int ret=system(cmd);
    if(ret!=0) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::sysCall SystemCall error occurred, cmd=\""+cmdStr+"\" rc="+std::to_string(ret));
    }
}

void
TCSimulatorItem::sysCall2(const std::string& cmdStr,const std::string& nic,const char *num) {
    char pre[256];
    sprintf(pre,"tc qdisc show dev %s",nic.c_str());

    char buf[10240];
    char *p=buf;
    FILE *fp;

    if((fp=popen(pre,"r"))!=NULL) {
        char line[1024];
        while(fgets(line,sizeof line,fp)!=NULL) {
            p+=sprintf(p,"%s",line);
        }
        pclose(fp);
    }

    char check[256];

    sprintf(check,"qdisc netem %s:",num);

    if(strstr(buf,check)==NULL) {
        sysCall(cmdStr);
    }
}

void
TCSimulatorItem::sysCall3(const std::string& cmdStr,const std::string& nic,const char *num) {
    char pre[256];
    sprintf(pre,"tc qdisc show dev %s",nic.c_str());

    char buf[10240];
    char *p=buf;
    FILE *fp;

    if((fp=popen(pre,"r"))!=NULL) {
        char line[1024];
        while(fgets(line,sizeof line,fp)!=NULL) {
            p+=sprintf(p,"%s",line);
        }
        pclose(fp);
    }

    char check[256];

    sprintf(check,"qdisc ingress %s:",num);

    if(strstr(buf,check)==NULL) {
        sysCall(cmdStr);
    }
}

int
TCSimulatorItem::listEther(const char *base_path) {
    DIR *dir=opendir(base_path);
    if (dir == NULL) {
        perror(base_path);
        return 0;
    }

    _allNIC.clear();
    struct dirent *dent;
    while ((dent = readdir(dir)) != NULL) {
        _allNIC.push_back(std::string(dent->d_name));
    }

    closedir(dir);

    return _allNIC.size();
}

bool
TCSimulatorItem::chkIPMask(const std::string& ip) {
    if(ip.compare("")==0) {
        return true;
    }

    bool rc=true;

    int len=ip.length();

    char ipMask[len+1];
    memcpy(ipMask,ip.c_str(),len+1);
    char *p=ipMask;

    int ipCnt=0;
    char *masks[256];
    masks[ipCnt]=p;
    ipCnt++;
    while(*p!='\0') {
        if(*p==IP_SEPARATOR) {
            if(ipCnt==256) {
                rc = false;
                break;
            }
            *p='\0';
            masks[ipCnt]=p+1;
            ipCnt++;
        }
        p++;
    }

    if(rc) {
        for(int j=0;j<ipCnt;j++) {
            char *arr[5];
            int cnt=0;
            p=masks[j];
            arr[0]=p;
            cnt++;
            while(*p!='\0') {
                if(*p=='.'||*p=='/') {
                    *p = '\0';
                    arr[cnt]=p+1;
                    cnt++;
                    if(cnt==5) {
                        break;
                    }
                }
                p++;
            }

            if(cnt!=5) {
                rc = false;
                break;
            }

            if(rc) {
                for(int i=0;i<cnt;i++) {
                    if(rc) {
                        int len=strlen(arr[i]);
                        if(1<=len&&len<=3) {

                        }
                        else {
                            rc=false;
                            break;
                        }

                        rc = chkDigits(arr[i]);
                    }
                    else {
                        break;
                    }
                    if(rc) {
                        int num=atoi(arr[i]);
                        if(i<4&&(num<0||num>255)) {
                            rc = false;
                        }
                        if(i==4&&(num<0||num>32)) {
                            rc = false;
                        }
                    }
                }
            }
        }
    }

    return rc;
}

bool
TCSimulatorItem::chkDigits(const char *p) {
    bool rc = true;

    while(*p!='\0') {
        if(*p>='0'&&*p<='9') {

        }
        else {
            rc=false;
            break;
        }
        p++;
    }

    return rc;
}

void
TCSimulatorItem::setVirNIC() {
    int portCount=_communicationPort.size();
    for(int i=0;i<portCount;i++) {
        _ethName[i] = _communicationPort.symbol(i);
        _virName[i] = _pair[_ethName[i]];
    }
}

void
TCSimulatorItem::doTC() {
    if(_enableTrafficControl==false) return;

    TrafficControlShare* _share = TrafficControlShare::instance();
    if(_share->getTcsRunning()==false) return;

    int portCount=_communicationPort.size();
    for(int i=0;i<portCount;i++) {
        doTC(i);
    }
}

void
TCSimulatorItem::initTC() {
    for(auto itr = _pair.begin(); itr!=_pair.end();++itr) {
        string cmd;
        string eth=itr->first;
        string ifb=itr->second;

        cmd="tc qdisc add dev "+eth+" ingress handle ffff:";
        sysCall3(cmd,eth,"ffff");

        cmd="tc filter add dev "+eth+" protocol ip parent ffff: prio 1 u32 match u32 0 0 flowid 1:1 action mirred egress redirect dev "+ifb;
        sysCall(cmd);
        cmd="tc filter add dev "+eth+" protocol ip parent ffff: prio 2 u32 match u32 0 0 flowid 1:2 action mirred egress redirect dev "+ifb;
        sysCall(cmd);

        cmd="tc qdisc replace dev "+eth+" root handle 1: prio";
        sysCall(cmd);

        cmd="tc filter add dev "+eth+" protocol ip parent 1: prio 1 u32 match ip dst 0.0.0.0/0 flowid 1:1";// action mirred egress redirect dev "+ifb;
        sysCall(cmd);
        cmd="tc filter add dev "+eth+" protocol ip parent 1: prio 2 u32 match ip dst 0.0.0.0/0 flowid 1:2";// action mirred egress redirect dev "+ifb;
        sysCall(cmd);

        cmd="tc qdisc add dev "+eth+" parent 1:1 handle 11: netem delay 0ms rate 0kbit loss 0%";
        sysCall2(cmd,eth,"11");
        cmd="tc qdisc add dev "+eth+" parent 1:2 handle 12: netem delay 0ms rate 0kbit loss 0%";
        sysCall2(cmd,eth,"12");

        cmd="tc qdisc replace dev "+ifb+" root handle 1: prio";
        sysCall(cmd);

        cmd="tc filter add dev "+ifb+" protocol ip parent 1: prio 1 u32 match ip src 0.0.0.0/0 flowid 1:1";
        sysCall(cmd);
        cmd="tc filter add dev "+ifb+" protocol ip parent 1: prio 2 u32 match ip src 0.0.0.0/0 flowid 1:2";
        sysCall(cmd);

        cmd="tc qdisc add dev "+ifb+" parent 1:1 handle 11: netem delay 0ms rate 0kbit loss 0%";
        sysCall2(cmd,ifb,"11");
        cmd="tc qdisc add dev "+ifb+" parent 1:2 handle 12: netem delay 0ms rate 0kbit loss 0%";
        sysCall2(cmd,ifb,"12");
    }
}

void
TCSimulatorItem::doTC(const int& ethIdxNo) {
    if(_ipAddressC[ethIdxNo].compare(_ipAddressT[ethIdxNo])==0) {
        doTC2S(ethIdxNo);
    }
    else {

        string cmd;

        cmd = "tc filter delete dev "+_ethName[ethIdxNo]+" parent 1: prio 1";
        sysCall(cmd);

        cmd = "tc filter delete dev "+_virName[ethIdxNo]+" parent 1: prio 1";
        sysCall(cmd);

        if(_ipAddressC[ethIdxNo].compare("")==0) {
            cmd="tc filter add dev "+_ethName[ethIdxNo]+" protocol ip parent 1: prio 1 u32 match ip dst "+"0.0.0.0/0"+" flowid 1:1";// action mirred egress redirect dev "+_virName[ethIdxNo];
            sysCall(cmd);

            cmd="tc filter add dev "+_virName[ethIdxNo]+" protocol ip parent 1: prio 1 u32 match ip src "+"0.0.0.0/0"+" flowid 1:1";
            sysCall(cmd);
        }
        else {
            vector<string> v=split(_ipAddressC[ethIdxNo],IP_SEPARATOR);

            int vsize=v.size();
            for(int i=0;i<vsize;i++) {
                cmd="tc filter add dev "+_ethName[ethIdxNo]+" protocol ip parent 1: prio 1 u32 match ip dst "+v.at(i)+" flowid 1:1";// action mirred egress redirect dev "+_virName[ethIdxNo];
                sysCall(cmd);

                cmd="tc filter add dev "+_virName[ethIdxNo]+" protocol ip parent 1: prio 1 u32 match ip src "+v.at(i)+" flowid 1:1";
                sysCall(cmd);
            }
        }
        _ipAddressT[ethIdxNo]=_ipAddressC[ethIdxNo];

        doTC2S(ethIdxNo);
    }
}

void
TCSimulatorItem::doTC2com(const int& ethIdxNo,const int *p1,const int *p2,const double *p3,const int *p4,const int *p5,const double *p6) {

    bool rc1=true;

    if(p1[ethIdxNo]<ZERO||p1[ethIdxNo]>DELAY_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com OutboundDelay value="+std::to_string(p1[ethIdxNo])+" is out of range."+rangeStr(ZERO,DELAY_MAX));
        rc1=false;
    }
    if(p2[ethIdxNo]<ZERO||p2[ethIdxNo]>BAND_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com OutboundBandWidth value="+std::to_string(p2[ethIdxNo])+" is out of range."+rangeStr(ZERO,BAND_MAX));
        rc1=false;
    }
    if(p3[ethIdxNo]<ZERO||p3[ethIdxNo]>LOSS_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com OutboundLoss value="+std::to_string(p3[ethIdxNo])+" is out of range."+rangeStr(ZERO,LOSS_MAX));
        rc1=false;
    }
    if(rc1) {
        string cmd="tc qdisc replace dev "+_ethName[ethIdxNo]+" parent 1:1 handle 11: netem"+" delay "+std::to_string(p1[ethIdxNo])+"ms"+" rate "+std::to_string(p2[ethIdxNo])+"kbit"+" loss "+std::to_string(p3[ethIdxNo])+"%";
        sysCall(cmd);
    }

    bool rc2=true;

    if(p4[ethIdxNo]<ZERO||p4[ethIdxNo]>DELAY_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com InboundDelay value="+std::to_string(p4[ethIdxNo])+" is out of range."+rangeStr(ZERO,DELAY_MAX));
        rc2=false;
    }
    if(p5[ethIdxNo]<ZERO||p5[ethIdxNo]>BAND_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com InboundBandWidth value="+std::to_string(p5[ethIdxNo])+" is out of range."+rangeStr(ZERO,BAND_MAX));
        rc2=false;
    }
    if(p6[ethIdxNo]<ZERO||p6[ethIdxNo]>LOSS_MAX) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"TCSimulatorItem::doTC2com InboundLoss value="+std::to_string(p6[ethIdxNo])+" is out of range."+rangeStr(ZERO,LOSS_MAX));
        rc2=false;
    }
    if(rc2) {
        string cmd="tc qdisc replace dev "+_virName[ethIdxNo]+" parent 1:1 handle 11: netem"+" delay "+std::to_string(p4[ethIdxNo])+"ms"+" rate "+std::to_string(p5[ethIdxNo])+"kbit"+" loss "+std::to_string(p6[ethIdxNo])+"%";
        sysCall(cmd);
    }
}

void
TCSimulatorItem::doTC2S(const int& ethIdxNo) {
    doTC2com(ethIdxNo,_OutboundDelay,_OutboundBandWidth,_OutboundLoss,_InboundDelay,_InboundBandWidth,_InboundLoss);
}

void
TCSimulatorItem::doTC2D(const int& ethIdxNo) {
    doTC2com(ethIdxNo,_OutboundDelayD,_OutboundBandWidthD,_OutboundLossD,_InboundDelayD,_InboundBandWidthD,_InboundLossD);
}

vector<string>
TCSimulatorItem::split(const string &str,const char& delim) {
    vector<string> elems;
    stringstream ss(str);
    string item;
    while (getline(ss, item, delim)) {
        if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

int
TCSimulatorItem::getEthIndexNo(const string &ethName) {
    int idx=-1;

    int portCount=_communicationPort.size();

    for(int i=0;i<portCount;i++) {
        if(ethName.compare(_ethName[i])==0) {
            idx = i;
            break;
        }
    }

    return idx;
}

void
TCSimulatorItem::bridgeInit() {
    int portCount=_communicationPort.size();

    for(int i=0;i<portCount;i++) {
        _OutboundDelay[i]=0;
        _OutboundBandWidth[i]=0;
        _OutboundLoss[i]=0;
        _InboundDelay[i]=0;
        _InboundBandWidth[i]=0;
        _InboundLoss[i]=0;
        _ipAddressT[i] = "xyz";
        doTC(i);
    }

    for(int i=0;i<portCount;i++) {
        doTC2D(i);
    }
}

void
TCSimulatorItem::bridgeTC(const int& ethIdxNo,const double& distance,const double& upDelay, const double& upRate, const double& upLoss, const double& dnDelay, const double& dnRate, const double& dnLoss){

    if(ethIdxNo==-1||ethIdxNo>=NIC_MAX) return;

    int i1 = upDelay;
    int i2 = upRate;
    double i3 = upLoss;
    int i4 = dnDelay;
    int i5 = dnRate;
    double i6 = dnLoss;

    if(i1<0) return;
    if(i2<0) return;
    if(i3<0) return;
    if(i4<0) return;
    if(i5<0) return;
    if(i6<0) return;

    if(i1>DELAY_MAX) {
        i1=DELAY_MAX;
    }
    if(i2>BAND_MAX) {
        i2=BAND_MAX;
    }
    if(i3>LOSS_MAX) {
        i3=LOSS_MAX;
    }
    if(i4>DELAY_MAX) {
        i4=DELAY_MAX;
    }
    if(i5>BAND_MAX) {
        i5=BAND_MAX;
    }
    if(i6>LOSS_MAX) {
        i6=LOSS_MAX;
    }

    if(_monitorDynamicTC) {
        static double pTime[NIC_MAX];

        double cTime=_curSimItem->currentTime();
        if(cTime-pTime[ethIdxNo]<0.002&&pTime[ethIdxNo]<cTime) {

        }
        else {
            char buf[1024];
            sprintf(buf,"Debug:,|,%s,%7.3fs,%7.3fM,|,%6dms,%6dkbit,%6.2f%%,|,%6dms,%6dkbit,%6.2f%%",_ethName[ethIdxNo].c_str(),cTime,distance,i1,i2,i3,i4,i5,i6);
            MessageView::mainInstance()->putln(std::string(buf));
        }

        pTime[ethIdxNo]=cTime;
    }

    if(i1==_OutboundDelayD[ethIdxNo] &&
            i2 ==_OutboundBandWidthD[ethIdxNo] &&
            i3 ==_OutboundLossD[ethIdxNo] &&
            i4 ==_InboundDelayD[ethIdxNo] &&
            i5 ==_InboundBandWidthD[ethIdxNo] &&
            i6 ==_InboundLossD[ethIdxNo]) {
    }
    else {
        _OutboundDelayD[ethIdxNo] = i1;
        _OutboundBandWidthD[ethIdxNo] = i2;
        _OutboundLossD[ethIdxNo] = i3;
        _InboundDelayD[ethIdxNo] = i4;
        _InboundBandWidthD[ethIdxNo] = i5;
        _InboundLossD[ethIdxNo] = i6;

        doTC2D(ethIdxNo);
    }
}

