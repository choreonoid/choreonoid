/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include "MulticopterSimulatorItem.h"
#include <cnoid/YAMLBodyLoader>
#include <fmt/format.h>
#include <cmath>
#include <random>

using namespace std;
using namespace cnoid;
using fmt::format;
using namespace Multicopter;


const double DEFAULT_GRAVITY_ACCELERATION = 9.80665;

SimulationManager* SimulationManager::_inst = 0;

SimulationManager*
SimulationManager::instance()
{
    if( _inst == 0 ){
        _inst = new SimulationManager();
    }
    return _inst;
}

bool
SimulationManager::initialize(ExtensionManager* extMgr)
{

    extMgr->viewManager().registerClass<MulticopterMonitorView>("MulticopterMonitorView",_("MultiCopterMonitor"), ViewManager::SINGLE_OPTIONAL);
    _multicopterMonitorView = extMgr->viewManager().findView<MulticopterMonitorView>();

    _gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;

    std::function<void(bool,double,SimulatorItem*, SubSimulatorItem*)> simStartEndEv = std::bind(&SimulationManager::onSimulationStartEnd, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    _simStartEndEvId = EventManager::instance()->addSimulationStartEndEvent(simStartEndEv);

    std::function<void(double,SimulatorItem*, SubSimulatorItem*)> simStepEv = std::bind(&SimulationManager::onSimulationStep, this, std::placeholders::_1,std::placeholders::_2, std::placeholders::_3);
    _simStepEvId = EventManager::instance()->addSimulationStepEvent(simStepEv);

    std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)> bodyCreateEv = std::bind(&SimulationManager::onBodyCreate,this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    _bodyCreateEvId = LinkManager::instance()->addBodyCreateEvent(bodyCreateEv);

    std::function<void(const QUuid&, const cnoid::Body*, const std::vector<cnoid::Link*>&, const std::vector<cnoid::Device*>&)> bodyDeleteEv = std::bind(&SimulationManager::onBodyDelete, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4);
    _bodyDeleteEvId = LinkManager::instance()->addBodyDeleteEvent(bodyDeleteEv);

    return true;
}


bool
SimulationManager::isInitialized() const
{
    return _isInitialized;
}

FluidEnvironment*
SimulationManager::fluidEnvironment()
{
    return _fluEnv;
}

FluidEnvironment*
SimulationManager::fluidEnvironmentSim()
{
    return _fluEnvSim;
}

void
SimulationManager::setNewFluidEnvironment(){
    _fluEnv =new FluidEnvironment();
}


SimulationManager::SimulationManager()
{
    _isInitialized = false;

    _fluEnv = new FluidEnvironment();
    _fluEnvSim = new FluidEnvironment();

    _enableLinkForceDump = false;
    _degree=4;

    _fluidDensitySim=_fluidDensity=0;
    _viscositySim= _viscosity=0;
    _fluidVelocitySim=_fluidVelocity= Vector3(0,0,0);
    _wallEffectSim=_wallEffect=false;
    _groundEffectSim=_groundEffect=false;
    _logFlgSim=_logFlg=false;
    _logIntrvSim=_logIntrv=0.001;

    _effectLinkBodyMapSize=0;

}

SimulationManager::~SimulationManager()
{
    delete _fluEnv;
}

bool
SimulationManager::initializeSimulation(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    _isInitialized = false;

    multicopterSimItem->setParameterToSimulationManager();

    _fluidDensitySim=_fluidDensity;
    _viscositySim= _viscosity;
    _fluidVelocitySim=_fluidVelocity;
    *_fluEnvSim=*_fluEnv;
    _wallEffectSim=_wallEffect;
    _groundEffectSim=_groundEffect;
    _logFlgSim=_logFlg;
    _logIntrvSim=_logIntrv;

    _fluEnvAllSim.density=_fluidDensitySim;
    _fluEnvAllSim.viscosity=_viscositySim;
    _fluEnvAllSim.velocity=_fluidVelocitySim;
    if(_fluidDensitySim<=0 || _viscosity<=0)_fluEnvAllSim.isFluid=false;
    else _fluEnvAllSim.isFluid=true;

    _collisionDetector= new AISTCollisionDetector;

    string cmmMsg = _("Could not execute Simulation.");
    if( fluidEnvironmentSim()->isNull() == true && (_fluidDensitySim<=0 || _viscositySim <=0)){
        UtilityImpl::printErrorMessage(_("Fluid Environment is not ready."));
        UtilityImpl::printErrorMessage(cmmMsg);
        return false;
    }

    list<Body*> bodyList = fluidDynamicsTargetBodies(simItem);

    updateBodyLinkMap(bodyList);

    _gravity=simItem->getGravity();

    clearLinkPolygon();

    vector<Link*> lnkAry;
    lnkAry.reserve(_fluidLinkBodyMap.size());
    for(auto it=begin(_fluidLinkBodyMap); it !=end(_fluidLinkBodyMap);++it){
        lnkAry.push_back(const_cast<Link*>(it->first));
    }
    updateLinkPolygon(lnkAry);

    calculateSurfaceCuttoffCoefficient(_fluidLinkBodyMap,_linkPolygonMap);

    double curTime = simItem->currentTime();
    _nextLogTime         = curTime;

    if( _logFlg == true ){
        MulticopterMonitorView* linkView = multicopterMonitorView();
        linkView->clear();
        linkView->writeln(MULTICOPTER_LOG_HEADER);
    }

    updateLinkState(curTime);

    #ifdef ENABLE_MULTICOPTER_PLUGIN_DEBUG
        debugList();
    #endif

    _isInitialized = true;

    return true;
}

list<Body*>
SimulationManager::simulationTargetBodies(cnoid::SimulatorItem* simItem)
{
    const std::vector<SimulationBody*>& simBodyAry = simItem->simulationBodies();

    list<Body*> trgBdList;
    for(auto it = begin(simBodyAry) ; it != end(simBodyAry) ; ++it){
        Body* bd = (*it)->body();
        trgBdList.push_back(bd);
    }
    return trgBdList;
}

list<Body*>
SimulationManager::fluidDynamicsTargetBodies(cnoid::SimulatorItem* simItem)
{
    const std::vector<SimulationBody*>& simBodyAry = simItem->simulationBodies();

    list<Body*> trgBdList;
    for(auto it = begin(simBodyAry) ; it != end(simBodyAry) ; ++it){
        Body* bd = (*it)->body();
        Mapping* mapInfo = bd->info();
        if( mapInfo != nullptr ){
            Mapping* bodyAttr=UtilityImpl::mapping(mapInfo, YAML_BODY_TAG);
            bool flg=UtilityImpl::exist(mapInfo, YAML_BODY_TAG);
            if(bodyAttr!=nullptr || flg){
                trgBdList.push_back(bd);
            }
        }
    }
    return trgBdList;
}


bool
SimulationManager::isLogEnabled() const
{
    return _logFlg;
}

void
SimulationManager::setLogEnabled(bool flg)
{
    _logFlg = flg;
}

void
SimulationManager::setLogInterval(double sec)
{
    if( sec < 1.0e-4 ){
        return;
    }
    _logIntrv = sec;
}

void
SimulationManager::updateBodyLinkMap(const std::list<Body*>& bodyList)
{
    clearBodyLinkMap();

    LinkManager* lnkMgr = LinkManager::instance();

    for(auto it = begin(bodyList) ; it != end(bodyList) ; ++it){
        Body* body = *it;
        vector<Link*> linkAry;

        lnkMgr->linkArray(body, linkAry);
        _bodyLinkMap[const_cast<Body*>(body)] = linkAry;

        map<const Link*, LinkAttribute> linkAttrMap;
        UtilityImpl::linkAttributeFromBodyMappingInfo(const_cast<Body*>(body), linkAttrMap);

        for(auto itl = begin(linkAttrMap) ; itl !=end(linkAttrMap) ; ++itl){
            auto handle = *_collisionDetector->addGeometry(const_cast<Link*>(itl->first)->collisionShape());
            itl->second.setGeometryHandle(handle);

            _linkBodyMap[const_cast<Link*>(itl->first)] = std::make_tuple(const_cast<Body*>(body), itl->second);

            if(itl->second.isNull()==false){
                _fluidLinkBodyMap[const_cast<Link*>(itl->first)] = std::make_tuple(const_cast<Body*>(body), itl->second);
                continue;
            }
            if(itl->second.effectMode()==2){
                _effectLinkBodyMap[const_cast<Link*>(itl->first)] = std::make_tuple(const_cast<Body*>(body), itl->second);
                continue;
            }
        }
    }

    _collisionDetector->makeReady();
    _effectLinkBodyMapSize=_effectLinkBodyMap.size();

}

void
SimulationManager::clearBodyLinkMap()
{
    _bodyLinkMap.clear();
    _linkBodyMap.clear();
    _fluidLinkBodyMap.clear();
    _effectLinkBodyMap.clear();
}

void
SimulationManager::updateLinkPolygon(const vector<Link*>& linkAry)
{
    LinkManager* lnkMgr = LinkManager::instance();
    for(auto it = begin(linkAry) ; it != end(linkAry) ; ++it){
        vector<Triangle3d> triAry;
        lnkMgr->mesh(*it, triAry);
        vector<LinkTriangleAttribute> triAttrAry;
        triAttrAry.reserve(triAry.size());
        for(auto itl = begin(triAry) ; itl != end(triAry) ; ++itl){
            triAttrAry.emplace_back(*itl);
        }
        _linkPolygonMap[*it] = triAttrAry;
    }
}

LinkAttribute
SimulationManager::linkAttribute(const cnoid::Link* link) const
{
    auto ret = _linkBodyMap.find(const_cast<Link*>(link));
    return std::get<1>(ret->second);
}

const std::vector<LinkTriangleAttribute>&
SimulationManager::linkPolygon(const cnoid::Link* link) const
{
    auto ret = _linkPolygonMap.find(const_cast<Link*>(link));
    return ret->second;
}

void
SimulationManager::clearLinkPolygon()
{
    _linkPolygonMap.clear();
}

void
SimulationManager::updateLinkState(double time)
{
    _linkStateMap.clear();

    for(auto itb = begin(_bodyLinkMap) ; itb != end(_bodyLinkMap) ; ++itb){
        std::vector<cnoid::Link*>& linkAry = itb->second;
        for(auto itl = begin(linkAry) ; itl != end(linkAry) ; ++itl){

            FFCalc::LinkStatePtr pLinkState (
                new FFCalc::LinkState (time, (**itl)));
            _linkStateMap.insert (std::make_pair (*itl, pLinkState));
        }
    }
}

void
SimulationManager::clearLinkState()
{
    _linkStateMap.clear();
}

void
SimulationManager::finalizeSimulation(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    if( multicopterSimItem == nullptr ){
        return;
    }

    clearBodyLinkMap();
    clearLinkPolygon();
    clearLinkState();
}

void
SimulationManager::onSimulationStartEnd(bool flg, double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem)
{
    if( flg == true ){
        initializeSimulation(simItem, dynamic_cast<MulticopterSimulatorItem*>(subSimItem));
    }
    else{
        finalizeSimulation(simItem, dynamic_cast<MulticopterSimulatorItem*>(subSimItem));
    }
}

void
SimulationManager::onSimulationStep(double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem)
{

}

void
SimulationManager::onBodyCreate(const QUuid& id, const cnoid::Body* body, const std::vector<cnoid::Link*>& linkAry, const std::vector<cnoid::Device*>& devAry)
{
    if( id != _bodyCreateEvId ){
        return;
    }
    map<const Link*, LinkAttribute> linkAttrMap;
    UtilityImpl::linkAttributeFromBodyMappingInfo(const_cast<Body*>(body), linkAttrMap);

    for(auto& linkAttr : linkAttrMap){
        _nonSimLinkBodyMap[const_cast<Link*>(linkAttr.first)] = make_tuple(const_cast<Body*>(body), linkAttr.second);
    }
}

void
SimulationManager::onBodyDelete(const QUuid& id, const cnoid::Body* body, const std::vector<cnoid::Link*>& linkAry, const std::vector<cnoid::Device*>& devAry)
{
    if( id != _bodyDeleteEvId ){
        return;
    }

    for(auto& link : linkAry){
        _nonSimLinkBodyMap.erase(link);
    }
}

void
SimulationManager::calculateSurfaceCuttoffCoefficient(map<Link*, tuple<Body*, LinkAttribute>>& fluidLinkBodyMap, map<Link*, vector<LinkTriangleAttribute>>& linkPolygonMap)
{
    const size_t numIP = getDegreeNumber();

    const size_t numLink = linkPolygonMap.size();

    vector<Link*> linkAry;
    vector<vector<FFCalc::GaussTriangle3d>> triAryList;
    linkAry.reserve(numLink);
    triAryList.reserve(numLink);

    for(auto& linkPolygon : linkPolygonMap){
        Link& link = *(linkPolygon.first);
        vector<LinkTriangleAttribute>& triAttrAry = linkPolygon.second;
        const size_t numTri = triAttrAry.size();

        vector<FFCalc::GaussTriangle3d> triAry;
        triAry.reserve(numTri);

        for(auto& triAttr : triAttrAry){
            triAry.push_back (FFCalc::GaussTriangle3d (triAttr.triangle(), link.T()));
        }

        linkAry.push_back(&link);
        triAryList.push_back(triAry);
    }

    for(size_t i=0 ; i<numLink ; ++i){

        Link& curLink = *(linkAry[i]);
        const vector<FFCalc::GaussTriangle3d>& curTriAry = triAryList[i];

        const LinkAttribute& linkAttr = get<1>(fluidLinkBodyMap[&curLink]);
        double cutoffDist = linkAttr.cutoffDistance();
        double normMidVal = linkAttr.normMiddleValue();
        FFCalc::CutoffCoef cutoffCalc(cutoffDist, normMidVal);
        int curTriArySize=curTriAry.size();

        for(size_t j=0 ; j<curTriArySize ; ++j){

            const FFCalc::GaussTriangle3d& curTri = curTriAry[j];

#ifdef _WIN32
            vector<double> minCoefs(numIP);
#else
            double minCoefs[numIP];
#endif
            for(int it=0;it<numIP;it++)minCoefs[it]=1.0;

            for(size_t k=0 ; k<numLink ; ++k){

                Link& trgLink = *(linkAry[k]);
                const vector<FFCalc::GaussTriangle3d>& trgTriAry = triAryList[k];

                if( &curLink == &trgLink )
                    continue;

                if( trgTriAry.empty() == true )
                    continue;

#ifdef _WIN32
                double* coefs = new double[numIP];
#else
                double coefs[numIP];
#endif
                calcCuttoffCoef(cutoffCalc, curTri, trgTriAry, coefs);
                for(int iIP=0 ; iIP<numIP ; ++iIP){
                    if( coefs[iIP] < minCoefs[iIP] ){
                        minCoefs[iIP] = coefs[iIP];
                    }
                }
#ifdef _WIN32
                delete[] coefs;
#endif
            }

            LinkTriangleAttribute& triAttr = linkPolygonMap[&curLink][j];
            for(int iIP=0 ; iIP<numIP ; ++iIP){
                triAttr.setCutoffoefficient(iIP, minCoefs[iIP]);
            }
        }
    }
}

void
SimulationManager::calcCuttoffCoef (
        const FFCalc::CutoffCoef& cutoffCalc,
        const FFCalc::GaussTriangle3d& tri,
        const std::vector<FFCalc::GaussTriangle3d>& trgTriAry,
        double coefs[])
{
    int numIP = getDegreeNumber();
    for(int it=0;it<numIP;it++)coefs[it]=1.0;
    for(size_t iIP=0 ; iIP<numIP ; ++iIP){
        const Eigen::Vector3d point = tri.getGaussPoint(iIP,numIP);
        int trgTriArySize=trgTriAry.size();
        for(size_t i=0 ; i<trgTriArySize ; ++i){
            double coef = cutoffCalc.get (point, tri.normal(), trgTriAry[i]);
            if( coef < coefs[iIP] ){
                coefs[iIP] = coef;
            }
        }
    }
}

void
SimulationManager::preDynamicFunction(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    if(multicopterSimItem = nullptr){
        return;
    }

    if( isInitialized() == false ){
        return;
    }
}

void
SimulationManager::midDynamicFunction(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    if(multicopterSimItem = nullptr){
        return;
    }

    if( isInitialized() == false ){
        return;
    }

    _rotorOutValAry.clear();
    _linkOutValAry.clear();

    for(auto itb = begin(_bodyLinkMap) ; itb != end(_bodyLinkMap) ; ++itb){
        std::map<int,std::tuple<double,Vector3>> effectMap;
        bool calFlag=false;


        const LinkAttribute& rootLinkAttr = linkAttribute(itb->first->rootLink());
        std::vector<cnoid::Link*>& linkAry = itb->second;

        if(rootLinkAttr.effectMode()==1 && _effectLinkBodyMapSize > 0 && (_wallEffectSim || _groundEffectSim)){
            calFlag=true;
            Vector3 v1, v2,direction;

            for(auto itl = begin(_effectLinkBodyMap) ; itl !=end(_effectLinkBodyMap) ; ++itl){
                int size=effectMap.size();
                double min=-1.0;
                CollisionDetector::GeometryHandle handle2=linkAttribute(const_cast<Link*>(itl->first)).getGeometryHandle().get();
                _collisionDetector->updatePosition(handle2,const_cast<Link*>(itl->first)->position());

                for(auto itll = begin(linkAry) ; itll != end(linkAry) ; ++itll){
                    CollisionDetector::GeometryHandle handle1=linkAttribute(const_cast<Link*>(*itll)).getGeometryHandle().get();
                    _collisionDetector->updatePosition(handle1,const_cast<Link*>(*itll)->position());
                    double d = _collisionDetector->detectDistance(handle1,handle2,v1,v2);

                    if(min==-1.0 || d<min){
                        min=d;
                        direction=v2-v1;
                        effectMap[size]=std::make_tuple(min,direction);
                    }
                }
            }
        }

        for(auto itl = begin(linkAry) ; itl != end(linkAry) ; ++itl){

            try{
                FFCalc::LinkStatePtr pLinkState;
                pLinkState = _linkStateMap[*itl];
                pLinkState->update (simItem->currentTime(), **itl);

                std::unique_ptr<FFCalc::LinkForce> pLinkForce = midDynamicFunctionLink (
                    simItem, multicopterSimItem, **itl, *pLinkState,effectMap,calFlag);

                (*itl)->f_ext()   += pLinkForce->getForce();
                (*itl)->tau_ext() += pLinkForce->getMoment();

            }
            catch(runtime_error& err){
                UtilityImpl::printErrorMessage(
                    format("{0:s} in {1:s} at {2:lf}",
                           err.what(), (*itl)->name(), simItem->currentTime()));
                continue;
            }
        }
    }
}


std::unique_ptr<FFCalc::LinkForce> SimulationManager::midDynamicFunctionLink (
    SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem, cnoid::Link& link, const FFCalc::LinkState& linkState,
    std::map<int,std::tuple<double,Vector3>> effectMap,bool calFlag=false)
{

    const Eigen::Vector3d loadingPoint = Vector3 (0.0, 0.0, 0.0);

    std::unique_ptr<FFCalc::LinkForce> pLinkForce(new FFCalc::LinkForce(loadingPoint));

    const FluidEnvironment& fluidEnv = *fluidEnvironmentSim();
    const LinkAttribute& linkAttr = linkAttribute(&link);

    if(linkAttr.isNull()==false){

        const std::vector<LinkTriangleAttribute>& triAttrAry = _linkPolygonMap[&link];
        const std::vector<bool>linkForceApplyTarget = linkAttr.linkForceApplyFlgAry();

        FFCalc::FFCalculator ffc (_gravity, fluidEnv, _fluEnvAllSim,link, linkAttr, linkState, triAttrAry);

        FFCalc::LinkForce lfBuoyancy(pLinkForce->point());
        if(linkForceApplyTarget[0] == true){
            ffc.calcBuoyancy(&lfBuoyancy);
            pLinkForce->add(lfBuoyancy);
        }

        FFCalc::LinkForce lfAddMass(pLinkForce->point());
        FFCalc::LinkForce lfAddMoment(pLinkForce->point());
        if(( linkForceApplyTarget[1] == true) || ( linkForceApplyTarget[2] == true)){
            ffc.calcAddMass (&lfAddMass, &lfAddMoment);
            if( linkForceApplyTarget[1] == true){
                pLinkForce->add(lfAddMass);
                if(linkForceApplyTarget[2] == true){
                    pLinkForce->add(lfAddMoment);
                }
            }else{
                pLinkForce->add(lfAddMoment);
            }
        }

        FFCalc::LinkForce lfSurface(pLinkForce->point());
        FFCalc::LinkForce lfGenSurface(pLinkForce->point());

        if(linkForceApplyTarget[3] == true){
            ffc.calcSurfaceGeneral (&lfGenSurface, &lfGenSurface,getDegreeNumber());
            lfSurface.add(lfGenSurface);
        }
        pLinkForce->add(lfSurface);


        FluidOutValue fluOutVal;
        fluOutVal.logMode      = linkAttr.logMode();
        fluOutVal.linkName     = link.name();
        fluOutVal.bodyName     = link.body()->name();
        fluOutVal.position     = link.R()*link.c()+link.p();
        fluOutVal.velocity     = linkState.translationalVelocityAt(fluOutVal.position);
        fluOutVal.accelerationRaw  = linkState.translationalRawAccelerationAt(fluOutVal.position);
        fluOutVal.acceleration     = linkState.translationalAccelerationAt(fluOutVal.position);
        fluOutVal.rotationalVelocity =linkState.rotationalVelocity();
        fluOutVal.rotationalAccelerationRaw =linkState.rotationalRawAcceleration();
        fluOutVal.rotationalAcceleration =linkState.rotationalAcceleration();
        fluOutVal.buoyancyForce    = lfBuoyancy.getForce();
        fluOutVal.addMassForce     = lfAddMass.getForce();
        fluOutVal.addInertiaTorque = lfAddMoment.getMoment();
        fluOutVal.surfaceForce     = lfSurface.getForce();

        _linkOutValAry.push_back(fluOutVal);



    #ifdef ENABLE_MULTICOPTER_PLUGIN_DEBUG
            if( _enableLinkForceDump == true ){
                Debug::printLinkForceInformation(simItem->currentTime(), &link, *pLinkForce);
            }
    #endif

    }

    list<RotorDevice*> rotorAry = targetRotorDevices(&link);
    for(auto& rotor : rotorAry){

        if(rotor->isActive()==false)continue;

        FluidEnvironment::FluidValue fluVal;
        Eigen::Vector3d rotorPos = linkState.toGlobalPosition(rotor->position());

        Eigen::Vector3d rotorDir = linkState.toGlobalDirection(rotor->direction());

        bool inBounds = fluidEnvironmentSim()->get(rotorPos, fluVal);
        bool calcFlag=true;

        if(inBounds==true){
            if(fluVal.isFluid ==true){
                calcFlag=true;
            }else calcFlag=false;
        }else if(_fluEnvAllSim.isFluid == true){
            calcFlag=true;
        }else calcFlag=false;

        if( calcFlag ==true){

            Eigen::Vector3d rotorForce =rotorDir;
            FFCalc::LinkForce lfRotor(pLinkForce->point());

            lfRotor.addForce (rotor->value()*rotorForce, rotorPos);
            lfRotor.addMoment(rotor->torque()*rotorDir);

            double wallEffectForce=0;
            double groundEffectForce=0;
            pLinkForce->add(lfRotor);

            if(calFlag==true){
                FFCalc::LinkForce power1(pLinkForce->point());
                FFCalc::LinkForce power2(pLinkForce->point());
                for(auto effect:effectMap){
                    if(_groundEffectSim==true){
                        if(std::get<0>(effect.second) < rotor->wallEffectDistance()){
                            FFCalc::CutoffCoefImpl* func= FFCalc::CutoffCoefImpl::createInstance(rotor->wallEffectDistance(),rotor->wallEffectNormMiddleValue());
                            double rate=func->eval(rotor->wallEffectDistance() - std::get<0>(effect.second)) * rotor->wallEffectMaxRate();
                            double cosine;
                            double directionChange=1;
                            double distanceNorm=std::get<1>(effect.second).norm();
                            if(std::get<0>(effect.second)>0 && distanceNorm>0){
                                cosine=std::get<1>(effect.second).dot(-rotorDir)/(distanceNorm*rotorDir.norm());
                                if(cosine>0)directionChange=-1;
                            }else cosine=1.0;

                            if(cosine<0)cosine*=-1;

                            if(std::get<0>(effect.second)>0 && distanceNorm>0){
                                power1.addForce(std::get<1>(effect.second).normalized()*rotor->value()*cosine*rate*directionChange, rotorPos);
                            }else {
                                power1.addForce (-rotorDir*rotor->value()*cosine*rate*directionChange, rotorPos);
                            }
                        }
                    }
                    if(_wallEffectSim==true){
                        if(std::get<0>(effect.second) < rotor->groundEffectDistance()){
                            FFCalc::CutoffCoefImpl* func= FFCalc::CutoffCoefImpl::createInstance(rotor->groundEffectDistance(),rotor->groundEffectNormMiddleValue());
                            double rate=func->eval(rotor->groundEffectDistance() - std::get<0>(effect.second) )* rotor->groundEffectMaxRate();
                            double sine;
                            double directionChange=1;
                            double distanceNorm=std::get<1>(effect.second).norm();
                            if(std::get<0>(effect.second)>0 && distanceNorm>0){
                                sine=(std::get<1>(effect.second).cross(-rotorDir)).norm()/(distanceNorm*rotorDir.norm());
                            }else sine=0.0;

                            if(sine<0)sine*=-1;

                            if(std::get<0>(effect.second)>0 && distanceNorm>0){
                                power2.addForce(std::get<1>(effect.second).normalized()*rotor->value()*sine*rate*directionChange, rotorPos);
                            }else {
                                power2.addForce(-rotorDir*rotor->value()*sine*rate*directionChange, rotorPos);
                            }
                        }
                    }
                    pLinkForce->add(power1);
                    pLinkForce->add(power2);
                    groundEffectForce=power1.getForce().norm();
                    wallEffectForce=power2.getForce().norm();
                }
            }
        }
    }

    return std::move(pLinkForce);
}

void
SimulationManager::postDynamicFunction(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    if(multicopterSimItem = nullptr){
        return;
    }

    if( isInitialized() == false ){
        return;
    }

    logProc(simItem, multicopterSimItem);

}


bool
SimulationManager::isLinkForceSyncDumpEnabled() const
{
    return _enableLinkForceDump;
}

void
SimulationManager::setLinkForceSyncDumpEnabled(bool flg)
{
    _enableLinkForceDump = flg;
}


void
SimulationManager::logProc(SimulatorItem* simItem, MulticopterSimulatorItem* multicopterSimItem)
{
    if( _logFlg == true ){
        double curTime = simItem->currentTime();
        MulticopterMonitorView* linkView = multicopterMonitorView();

        if( curTime >= _nextLogTime - _logIntrv*1.0e-6 ){
                for(auto& linkOutVal : _linkOutValAry){
                    if(linkOutVal.logMode == false)continue;
                    linkView->writeln(
                        format(" {:.4f},{:s},{:s}, {:.3e},{:.3e},{:.3e},"
                               " {:.3e},{:.3e},{:.3e}, {:.3e},{:.3e},{:.3e}, {:.3e},{:.3e},{:.3e},"
                               " {:.3e},{:.3e},{:.3e}, {:.3e},{:.3e},{:.3e}, {:.3e},{:.3e},{:.3e},"
                               " {:.3e},{:.3e},{:.3e}, {:.3e},{:.3e},{:.3e}",
                               curTime, linkOutVal.bodyName,linkOutVal.linkName,
                               linkOutVal.position.x(), linkOutVal.position.y(), linkOutVal.position.z(),
                               linkOutVal.velocity.x(), linkOutVal.velocity.y(), linkOutVal.velocity.z(),
                               linkOutVal.acceleration.x(), linkOutVal.acceleration.y(), linkOutVal.acceleration.z(),
                               linkOutVal.rotationalVelocity.x(), linkOutVal.rotationalVelocity.y(), linkOutVal.rotationalVelocity.z(),
                               linkOutVal.rotationalAcceleration.x(), linkOutVal.rotationalAcceleration.y(), linkOutVal.rotationalAcceleration.z(),
                               linkOutVal.buoyancyForce.x(), linkOutVal.buoyancyForce.y(), linkOutVal.buoyancyForce.z(),
                               linkOutVal.surfaceForce.x(), linkOutVal.surfaceForce.y(), linkOutVal.surfaceForce.z(),
                               linkOutVal.addMassForce.x(), linkOutVal.addMassForce.y(), linkOutVal.addMassForce.z(),
                               linkOutVal.addInertiaTorque.x(), linkOutVal.addInertiaTorque.y(), linkOutVal.addInertiaTorque.z()));
                }
            _nextLogTime += _logIntrv;
        }
    }
}

void
SimulationManager::setDegreeNumber(int degreeNumber)
{
    _degree=degreeNumber;
}

int
SimulationManager::getDegreeNumber() const
{
    return _degree;
}


list<RotorDevice*>
SimulationManager::targetRotorDevices() const
{
    list<RotorDevice*> ret;
    for(auto it = begin(_bodyLinkMap) ; it != end(_bodyLinkMap) ; ++it){
        DeviceList<RotorDevice> rotorDevAry = it->first->devices();
        ret.insert(end(ret), begin(rotorDevAry),end(rotorDevAry));
    }
    return ret;
}

list<RotorDevice*>
SimulationManager::targetRotorDevices(cnoid::Link* link) const
{
    list<RotorDevice*> retAry;
    auto ret = _linkBodyMap.find(link);
    if( ret == _linkBodyMap.end() ){
        return retAry;
    }

    Body* body = get<0>(ret->second);
    DeviceList<RotorDevice> rotorDevAry = body->devices();
    for(auto it = begin(rotorDevAry) ; it != end(rotorDevAry) ; ++it){
        if( (*it)->link() == link ){
            retAry.push_back(*it);
        }
    }
    return retAry;
}


void SimulationManager::setGroundEffect(bool groundEffect)
{
    _groundEffect = groundEffect;
}

void SimulationManager::setWallEffect(bool wallEffect)
{
    _wallEffect = wallEffect;
}

void SimulationManager::setFluidVelocity(const cnoid::Vector3 &fluidVelocity)
{
    _fluidVelocity = fluidVelocity;
}

void SimulationManager::setViscosity(double viscosity)
{
    _viscosity = viscosity;
}

void SimulationManager::setFluidDensity(double fluidDensity)
{
    _fluidDensity = fluidDensity;
}

void SimulationManager::debugList()
{
    debugMapLinkAttribute(_linkBodyMap);
    debugRotor();
}

void SimulationManager::debugMapLinkAttribute(std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> > linkmap)
{
    int mapSize=linkmap.size();
    std::cout<<mapSize<<endl;
    for(auto itl = begin(linkmap) ; itl !=end(linkmap) ; ++itl){
        std::cout<<const_cast<Link*>(itl->first)->name()<<std::endl;
        if(get<1>(itl->second).isNull())std::cout<<"T"<<std::endl;
        else std::cout<<"F"<<std::endl;
        std::cout<<get<1>(itl->second).effectMode()<<endl;
        std::cout<<get<1>(itl->second).cutoffDistance()<<endl;
        std::cout<<get<1>(itl->second).normMiddleValue()<<endl;
        std::cout<<get<1>(itl->second).density()<<endl;
        std::cout<<get<1>(itl->second).centerOfBuoyancy()<<endl;
        std::cout<<get<1>(itl->second).additionalMassCoef()<<endl;
        std::cout<<get<1>(itl->second).additionalInertiaMatrix()<<endl;
        std::vector<bool> linkForceApply=get<1>(itl->second).linkForceApplyFlgAry();
        for(int i=0;i<4;i++){
            if(linkForceApply[i])std::cout<<"T ";
            else std::cout<<"F ";
        }
        std::cout<<endl;
    }
    std::cout<<endl;

}

void SimulationManager::debugRotor(){

    list<RotorDevice*> rotorAry = targetRotorDevices();
    for(auto& rotor : rotorAry){
        std::cout<<rotor->name()<<std::endl;
        rotor->showParameter();
    }
}
