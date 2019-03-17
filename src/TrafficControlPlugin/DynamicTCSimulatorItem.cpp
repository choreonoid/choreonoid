/**
   @author Japan Atomic Energy Agency
*/

#include "DynamicTCSimulatorItem.h"
#include "TrafficControlShare.h"
#include <cnoid/SimulatorItem>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/Archive>
#include <cnoid/EigenUtil>
#include <cnoid/EigenArchive>
#include <dirent.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;

void
DynamicTCSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<DynamicTCSimulatorItem>("DynamicTCSimulatorItem");
    im.addCreationPanel<DynamicTCSimulatorItem>();
}

DynamicTCSimulatorItem::DynamicTCSimulatorItem()
{
    _curSimItem = nullptr;
    _preFuncId = _midFuncId = _postFuncId = -1;

    int cnt=0;

    _config=std::string(TC_CONF_FILE);
    char file[_config.size()+1];
    strcpy(file,_config.c_str());

    FILE *fp=fopen(file,"r");

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
                // nop
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
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem "+n1+" does not exist in this computer. Please check configure file="+_config);
                    }
                    if(rc2==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem "+n2+" does not exist in this computer. Please check configure file="+_config);
                    }
                    if(rc3==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem "+n1+" is already defined(duplicated). Please check configure file="+_config);
                    }
                    if(rc4==false) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem "+n2+" is already defined(duplicated). Please check configure file="+_config);
                    }
                }
            }
            lcnt++;
        }
        fclose(fp);
    }
    else {
        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem Configure file="+_config+" does not exist.");
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
            MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem NIC_MAX=="+std::to_string(NIC_MAX)+", so "+eth+" is not available. Please redefine NIC_MAX value more larger.");
        }
    }

    if(cnt>0) {
        _communicationPort.resize(cnt);
    }
    else {
        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::DynamicTCSimulatorItem Configure file="+_config+" is invalid. Effective Port is nothing.");
        string eth="No valid port exists.";
        _ethName[cnt]=eth;
        _communicationPort.setSymbol(0,eth);
        _communicationPort.resize(1);
    }

    for(int i=0;i<NIC_MAX;i++) {
        _dTCFlagC[i]=false;
        _dTCFlagP[i]=false;
        _referencePointC[i] = Vector3(0,0,0);
        _referencePointP[i] = Vector3(0,0,0);
        _targetBody[i] = "";
        _timeStep[i] = TSTEP_DEFAULT;
        _staticEthIndex[i] = -1;
    }
}

DynamicTCSimulatorItem::DynamicTCSimulatorItem(const DynamicTCSimulatorItem& org) : SubSimulatorItem(org)
{
    _communicationPort = org._communicationPort;
    for(int i=0;i<NIC_MAX;i++) {
        _dTCFlagC[i] = org._dTCFlagC[i];
        _dTCFlagP[i] = org._dTCFlagP[i];
        _referencePointC[i] = org._referencePointC[i];
        _referencePointP[i] = org._referencePointP[i];
        _targetBody[i] = org._targetBody[i];
        _timeStep[i] = org._timeStep[i];
        _ethName[i] = org._ethName[i];
        _staticEthIndex[i] = org._staticEthIndex[i];
    }
    _allNIC=org._allNIC;
}

DynamicTCSimulatorItem::~DynamicTCSimulatorItem()
{

}

Item*
DynamicTCSimulatorItem::doDuplicate() const
{
    return new DynamicTCSimulatorItem(*this);
}

void
DynamicTCSimulatorItem::checkBodyItem() {
    const std::vector<SimulationBody*>& simBodyAry = _curSimItem->simulationBodies();
    list<Body*> trgBdList;
    for(auto it = begin(simBodyAry) ; it != end(simBodyAry) ; ++it){
        Body* bd = (*it)->body();
        Mapping* mapInfo = bd->info();
        if( mapInfo != nullptr ){
            trgBdList.push_back(bd);
        }
    }

    int portCount=_communicationPort.size();
    for(int i=0;i<portCount;i++) {
        if(_dTCFlagC[i]==true) {
            bool find=false;

            for(auto it = begin(trgBdList); it!=end(trgBdList) ; ++it) {
                if((*it)->name().compare(_targetBody[i])==0) {
                    find=true;
                    _body[i]=*it;
                    _nextTime[i] = _timeStep[i];
                    break;
                }
            }

            if(find==false) {
                MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::checkBodyItem TargetBody = \""+_targetBody[i]+"\" does not exist in this project.");
                _dTCFlagC[i]=false;
            }
        }
    }
}

bool
DynamicTCSimulatorItem::chkEnables() {
    bool rc = false;
    int cnt = _communicationPort.size();
    for(int i=0;i<cnt;i++) {
        if(_dTCFlagC[i]==true) {
            rc = true;
            break;
        }
    }
    return rc;
}

bool
DynamicTCSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    if(chkEnables()==false) return false;

    _curSimItem = simulatorItem;

    _tcs = nullptr;
    TrafficControlShare* _share = TrafficControlShare::instance();
    if(_share->getTcsRunning()==true) {
        checkTcsInstance();
    }

    checkBodyItem();

    _midFuncId = simulatorItem->addMidDynamicsFunction(std::bind(&DynamicTCSimulatorItem::onMidDynamicFunction, this));

    if(_setSignal==false) {
        simulatorItem->sigSimulationPaused().connect(std::bind(&DynamicTCSimulatorItem::onPaused, this));
        simulatorItem->sigSimulationResumed().connect(std::bind(&DynamicTCSimulatorItem::onResumed, this));
        _setSignal = true;
    }

    return true;
}

void
DynamicTCSimulatorItem::finalizeSimulation()
{
    if(chkEnables()==false) return;

    _curSimItem->removeMidDynamicsFunction(_midFuncId);
    
    _curSimItem = nullptr;
}

void
DynamicTCSimulatorItem::onPaused() {
    if(chkEnables()==false) return;
}

void
DynamicTCSimulatorItem::onResumed() {
    if(chkEnables()==false) return;

    checkBodyItem();

    if(_tcs!=nullptr) {
        _tcs->bridgeInit();
    }
}

void
DynamicTCSimulatorItem::onPreDynamicFunction()
{

}

void
DynamicTCSimulatorItem::onMidDynamicFunction()
{
    if(chkEnables()==false) return;

    if(checkTcsInstance()==false) {
        return;
    }

    int portCount=_communicationPort.size();
    for(int i=0;i<portCount;i++) {
        if(_dTCFlagC[i]==true&&_staticEthIndex[i]!=-1) {
            if(_curSimItem->currentTime()>=_nextTime[i]) {
                Vector3 bodyPoint=_body[i]->rootLink()->position().translation();

                double distance=calcDistance(bodyPoint,_referencePointC[i]);

                bridgeTC(_staticEthIndex[i],distance);

                while(_curSimItem->currentTime()>_nextTime[i]) {
                    _nextTime[i]=_nextTime[i]+_timeStep[i];
                }
            }
        }
    }
}

void
DynamicTCSimulatorItem::onPostDynamicFunction()
{

}

void
DynamicTCSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{

    SubSimulatorItem::doPutProperties(putProperty);

    putProperty(_("Port"), _communicationPort,
                [&](int index){ _portChanged = true; _idxNew = index; return _communicationPort.selectIndex(index);});

    if(_portChanged) {
        putProperty(_("EnableDynamicTrafficControl"),_dTCFlagC[_idxNew], changeProperty(_dTCFlagC[_idxNew]));
        putProperty(_("ReferencePoint"), str(_referencePointC[_idxNew]), [&](const string& v){ return toVector3(v, _referencePointC[_idxNew]); });
        putProperty(_("TargetBody"),_targetBody[_idxNew], changeProperty(_targetBody[_idxNew]));
        putProperty.decimals(3).min(TSTEP_MIN);
        putProperty.decimals(3).max(TSTEP_MAX);
        putProperty(_("TimeStep[s]"), _timeStep[_idxNew], changeProperty(_timeStep[_idxNew]));

        _portChanged = false;
        _idxCur = _idxNew;
    }
    else {
        if(_dTCFlagC[_idxCur]==true&&_targetBody[_idxCur].compare("")==0) {
            if(_dTCFlagP[_idxCur]==true) {
                MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::doPutProperties TargetBody became empty, So EnableDynamicTrafficControl is changed to false automatically!");
            } else {
                MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::doPutProperties When EnableDynamicTrafficControl is changed to true, You must define TargetBody at first!");
            }
            _dTCFlagC[_idxCur] = false;
        }

        _dTCFlagP[_idxCur] = _dTCFlagC[_idxCur];

        double x = _referencePointC[_idxCur].x();
        double y = _referencePointC[_idxCur].y();
        double z = _referencePointC[_idxCur].z();
        if(REF_MIN<=x&&x<=REF_MAX&&REF_MIN<=y&&y<=REF_MAX&&REF_MIN<=z&&z<=REF_MAX) {
        }
        else {
            _referencePointC[_idxCur] = _referencePointP[_idxCur];
            char buf[1024];
            sprintf(buf,"DynamicTCSimulatorItem::doPutProperties RefferencePoint=(%f %f %f) has an error value. Each value must be between %f and %f.",x,y,z,REF_MIN,REF_MAX);
            MessageView::mainInstance()->putln(MessageView::ERROR,std::string(buf));
        }

        putProperty(_("EnableDynamicTrafficControl"),_dTCFlagC[_idxCur], changeProperty(_dTCFlagC[_idxCur]));
        putProperty(_("ReferencePoint"), str(_referencePointC[_idxCur]), [&](const string& v){ return toVector3(v, _referencePointC[_idxCur]); });
        putProperty(_("TargetBody"),_targetBody[_idxCur], changeProperty(_targetBody[_idxCur]));
        putProperty.decimals(3).min(TSTEP_MIN);
        putProperty.decimals(3).max(TSTEP_MAX);
        putProperty(_("TimeStep[s]"), _timeStep[_idxCur], changeProperty(_timeStep[_idxCur]));
    }

    _referencePointP[_idxCur] = _referencePointC[_idxCur];

}

bool
DynamicTCSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);

    int portCount=_communicationPort.size();
    archive.write("PortCount",portCount);

    string portName="";
    for(int i=0;i<portCount;i++){
        portName="Port"+std::to_string(i);
        archive.write(portName, _communicationPort.symbol(i));
        archive.write(portName+"EnableDynamicTrafficControl",_dTCFlagC[i]);
        cnoid::write(archive, portName+"ReferencePoint",_referencePointC[i]);
        archive.write(portName+"TargetBody",_targetBody[i]);
        archive.write(portName+"TimeStep", _timeStep[i]);
    }

    return true;
}

bool
DynamicTCSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);

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
                    MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore "+tmp+" does not exist in this computer.");
                }
                else if(rc2==false) {
                    MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore "+tmp+" does not defined in configure file="+_config);
                }

                if(rc1&&rc2) {
                    _communicationPort.setSymbol(cnt,tmp);
                    archive.read(portName+"EnableDynamicTrafficControl",_dTCFlagC[cnt]);
                    archive.read(portName+"EnableDynamicTrafficControl",_dTCFlagP[cnt]);
                    cnoid::read(archive, portName+"ReferencePoint",_referencePointC[cnt]);
                    archive.read(portName+"TargetBody",_targetBody[cnt]);
                    archive.read(portName+"TimeStep", _timeStep[cnt]);

                    if(_dTCFlagC[cnt]==true&&_targetBody[cnt].compare("")==0) {
                        _dTCFlagC[cnt] = false;
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore When "+tmp+":EnableDynamicTrafficControl is changed to true, You must define TargetBody at first!");
                    }

                    double x = _referencePointC[cnt].x();
                    double y = _referencePointC[cnt].y();
                    double z = _referencePointC[cnt].z();
                    if(REF_MIN<=x&&x<=REF_MAX&&REF_MIN<=y&&y<=REF_MAX&&REF_MIN<=z&&z<=REF_MAX) {

                    }
                    else {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore "+tmp+":RefferencePoint("+std::to_string(x)+" "+std::to_string(y)+" "+std::to_string(z)+") has an error value. Each value must be between "+std::to_string(REF_MIN)+" and "+std::to_string(REF_MAX)+" .");
                        _referencePointC[cnt] = Vector3(0,0,0);
                    }

                    if(_timeStep[cnt]<TSTEP_MIN||_timeStep[cnt]>TSTEP_MAX) {
                        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore "+tmp+":TimeStep value="+std::to_string(_timeStep[cnt])+" is out of range.(between "+std::to_string(TSTEP_MIN)+" and "+std::to_string(TSTEP_MAX)+")");
                        _timeStep[cnt] = TSTEP_DEFAULT;
                    }

                    cnt++;
                }
            }
            else {
                MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore NIC_MAX=="+std::to_string(NIC_MAX)+", Properties of "+tmp+" can not be restored. Please redefine NIC_MAX value more larger.");
            }
        }
        if(cnt>0) {
            _communicationPort.resize(cnt);
        }
        else {
            MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore This Project file is invalid. Effective Port is nothing.");
            cnt=0;
            for(auto itr = _pair.begin(); itr!=_pair.end();++itr) {
                string eth=itr->first;
                if(cnt<NIC_MAX) {
                    _ethName[cnt]=eth;
                    _communicationPort.setSymbol(cnt,eth);
                    cnt++;
                }
                else {
                    MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore NIC_MAX=="+std::to_string(NIC_MAX)+", so "+eth+" is not available. Please redefine NIC_MAX value more larger.");
                }
            }
            if(cnt>0) {
                _communicationPort.resize(cnt);
            }
            else {
                MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::restore This Project file is invalid and Configure file="+_config+" is invalid too.");
                string eth="No valid port exists.";
                _ethName[cnt]=eth;
                _communicationPort.setSymbol(0,eth);
                _communicationPort.resize(1);
            }
        }
    }

    return true;
}

int
DynamicTCSimulatorItem::listEther(const char *base_path) {
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

double
DynamicTCSimulatorItem::calcDistance(const cnoid::Vector3 &p1, const cnoid::Vector3 &p2) {
    double d;

    d = pow(p1.x()-p2.x(),2.0)+pow(p1.y()-p2.y(),2.0)+pow(p1.z()-p2.z(),2.0);

    return pow(d,0.5);
}

bool
DynamicTCSimulatorItem::checkTcsInstance(){
    TrafficControlShare* _share = TrafficControlShare::instance();
    if(_tcs==nullptr||_tcs!=_share->getTcsInstance()) {
        _tcs = _share->getTcsInstance();
    } else {
        return true;
    }

    if(_tcs==nullptr) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::checkTcsInstance TCSimulator does not exist in this project.");
        return false;
    }

    if(_tcs->isEnableTrafficControl()) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::checkTcsInstance EnableTrafficControl of TCSimulator is true. Please turn off to false at first!");
        return false;
    }

    if(_share->getTcsRunning()==false) {
        MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::checkTcsInstance TCSimulator does not exist in this project or it is not running.");
        return false;
    }

    int portCount=_communicationPort.size();
    for(int i=0;i<portCount;i++) {
        _staticEthIndex[i] = _tcs->getEthIndexNo(_communicationPort.symbol(i));
        if(_staticEthIndex[i]==-1) {
            MessageView::mainInstance()->putln(MessageView::ERROR,"DynamicTCSimulatorItem::checkTcsInstance "+_communicationPort.symbol(i)+" does not exist in TCSimulator.");
        } else {
            _staticEthName[_staticEthIndex[i]]=_communicationPort.symbol(i);
        }
    }

    _tcs->bridgeInit();

    return true;
}

bool
DynamicTCSimulatorItem::findNIC(const std::string &nic) {
    bool find=false;

    for(int j=0;j<_allNIC.size();j++) {
        if(_allNIC[j].compare(nic)==0) {
            find=true;
            break;
        }
    }

    return find;
}

void
DynamicTCSimulatorItem::bridgeTC(const int& ethIdxNo, const double &distance) {
    if(_tcs==nullptr ) {
        return;
    }

    if(_staticEthName[ethIdxNo].compare("eth0")==0) {
        double outDelay=0.0;

        if(distance>=10.0) {
            outDelay=200.0;
        }
        else {
            outDelay=200.0 * distance / 10.0;
        }

        _tcs->bridgeTC(ethIdxNo,distance,outDelay,0,0,0,0,0);
    }
    else {
        double inDelay=0.0;

        if(distance>=10.0) {
            inDelay=200.0;
        }
        else {
            inDelay=200.0 * distance / 10.0;
        }

        _tcs->bridgeTC(ethIdxNo,distance,0,0,0,inDelay,0,0);
    }
}
