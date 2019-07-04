/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <fmt/format.h>

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using namespace Multicopter;


void MulticopterSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<MulticopterSimulatorItem>("MulticopterSimulatorItem");
    im.addCreationPanel<MulticopterSimulatorItem>();
}


MulticopterSimulatorItem::MulticopterSimulatorItem()
{
    setName("MulticopterSimulator");

    _curSimItem = nullptr;
    _preFuncId = _midFuncId = _postFuncId = -1;
    _fluidDensity=1.293;
    _viscosity=1.7e-5;
    _fluidVelocity = Vector3(0,0,0);
    _airDefinitionFileName="";
    _wallEffect=false;
    _groundEffect=false;
    _outputParam=false;
    _timeStep=1.0;
}


MulticopterSimulatorItem::MulticopterSimulatorItem(const MulticopterSimulatorItem& org) : SubSimulatorItem(org)
{
    _fluidDensity=org._fluidDensity;
    _viscosity=org._viscosity;
    _fluidVelocity=org._fluidVelocity;
    _airDefinitionFileName=org._airDefinitionFileName;
    _wallEffect=org._wallEffect;
    _groundEffect=org._groundEffect;
    _outputParam=org._outputParam;
    _timeStep=org._timeStep;
}

MulticopterSimulatorItem::~MulticopterSimulatorItem()
{

}

Item*
MulticopterSimulatorItem::doDuplicate() const
{
    return new MulticopterSimulatorItem(*this);
}

bool
MulticopterSimulatorItem::initializeSimulation(SimulatorItem* simulatorItem)
{
    SimulationManager* simMgr = SimulationManager::instance();
    
   _curSimItem = simulatorItem;
    EventManager* evMgr = EventManager::instance();
    evMgr->onSimulationStartEnd(true, _curSimItem->currentTime(), _curSimItem, this);
    
    if( simMgr->isInitialized() == true ){
        _preFuncId = simulatorItem->addPreDynamicsFunction(std::bind(&MulticopterSimulatorItem::onPreDynamicFunction, this));
        _midFuncId = simulatorItem->addMidDynamicsFunction(std::bind(&MulticopterSimulatorItem::onMidDynamicFunction, this));
        _postFuncId = simulatorItem->addPostDynamicsFunction(std::bind(&MulticopterSimulatorItem::onPostDynamicFunction, this));
    }

    return true;
}

void
MulticopterSimulatorItem::finalizeSimulation()
{
    SimulationManager* simMgr = SimulationManager::instance();

    _curSimItem->removePreDynamicsFunction(_preFuncId);
    _curSimItem->removeMidDynamicsFunction(_midFuncId);
    _curSimItem->removePostDynamicsFunction(_postFuncId);
    
    EventManager* evMgr = EventManager::instance();
    evMgr->onSimulationStartEnd(false, _curSimItem->currentTime(), _curSimItem, this);
    
    _curSimItem = nullptr;
}

void
MulticopterSimulatorItem::onPreDynamicFunction()
{
    SimulationManager* simMgr = SimulationManager::instance();
    simMgr->preDynamicFunction(_curSimItem, this);
}

void
MulticopterSimulatorItem::onMidDynamicFunction()
{
   EventManager* evMgr = EventManager::instance();
   evMgr->onSimulationStep(_curSimItem->currentTime(),_curSimItem,this);
       
   SimulationManager* simMgr = SimulationManager::instance();
   simMgr->midDynamicFunction(_curSimItem, this);
}

void
MulticopterSimulatorItem::onPostDynamicFunction()
{
   SimulationManager* simMgr = SimulationManager::instance();
   simMgr->postDynamicFunction(_curSimItem, this);
}

void
MulticopterSimulatorItem::setParameterToSimulationManager()
{
    SimulationManager* simMgr =SimulationManager::instance();
    simMgr->setFluidDensity(_fluidDensity);
    simMgr->setViscosity(_viscosity);
    simMgr->setFluidVelocity(_fluidVelocity);
    simMgr->setWallEffect(_wallEffect);
    simMgr->setGroundEffect(_groundEffect);
    simMgr->setLogEnabled(_outputParam);
    simMgr->setLogInterval(_timeStep);
    FluidEnvironment* fluEnv = simMgr->fluidEnvironment();
    if(_airDefinitionFileName==""){
        simMgr->setNewFluidEnvironment();
        return;
    }
    bool ret = fluEnv->load(_airDefinitionFileName);
    if( ret == false ){
        simMgr->setNewFluidEnvironment();
        return;
    }
}


void
MulticopterSimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
{
    SubSimulatorItem::doPutProperties(putProperty);

    FilePathProperty moduleProperty(_airDefinitionFileName);

    putProperty.decimals(3).min(0.0);
    putProperty(MULTICOPTER_DENSITY,_fluidDensity, changeProperty(_fluidDensity));
    putProperty.decimals(6).min(0.0);
    putProperty(MULTICOPTER_VISCOSITY,_viscosity, changeProperty(_viscosity));
    putProperty.decimals(3).min(0.0);
    putProperty(MULTICOPTER_VELOCITY, str(_fluidVelocity), [&](const string& v){ return toVector3(v, _fluidVelocity); });
    putProperty(MULTICOPTER_AIRDEFINITION, moduleProperty,
                [&](const string& name){ _airDefinitionFileName=name; return true; });

    putProperty(MULTICOPTER_WALLEFFECT, _wallEffect, changeProperty(_wallEffect));
    putProperty(MULTICOPTER_GROUNDEFFECT, _groundEffect, changeProperty(_groundEffect));
    putProperty(MULTICOPTER_OUTPUT, _outputParam, changeProperty(_outputParam));
    putProperty(MULTICOPTER_TIMESTEP, _timeStep, changeProperty(_timeStep));

    if( _airDefinitionFileName==_airDefinitionFileNameP)return;

    SimulationManager* simMgr =SimulationManager::instance();
    if(_airDefinitionFileName==""){
        _airDefinitionFileNameP=_airDefinitionFileName="";
        simMgr->setNewFluidEnvironment();
        UtilityImpl::printMessage("Air Definition File reset.");
        return;
    }

    FluidEnvironment* fluEnv = simMgr->fluidEnvironment();
    bool ret = fluEnv->load(_airDefinitionFileName);
    if( ret == false ){
        _airDefinitionFileNameP=_airDefinitionFileName="";
        simMgr->setNewFluidEnvironment();
        return;
    }
    _airDefinitionFileNameP=_airDefinitionFileName;
    UtilityImpl::printMessage("Air Definition File read successfully.");
}

bool
MulticopterSimulatorItem::store(Archive& archive)
{
    SubSimulatorItem::store(archive);

    archive.write(MULTICOPTER_DENSITY,_fluidDensity);
    archive.write(MULTICOPTER_VISCOSITY,_viscosity);
    cnoid::write(archive, MULTICOPTER_VELOCITY,_fluidVelocity);
    archive.write(MULTICOPTER_AIRDEFINITION, _airDefinitionFileName);
    archive.write(MULTICOPTER_WALLEFFECT,_wallEffect);
    archive.write(MULTICOPTER_GROUNDEFFECT, _groundEffect);
    archive.write(MULTICOPTER_OUTPUT, _outputParam);
    archive.write(MULTICOPTER_TIMESTEP, _timeStep);

    return true;
}

bool
MulticopterSimulatorItem::restore(const Archive& archive)
{
    SubSimulatorItem::restore(archive);

    archive.read(MULTICOPTER_DENSITY,_fluidDensity);
    archive.read(MULTICOPTER_VISCOSITY,_viscosity);
    cnoid::read(archive, MULTICOPTER_VELOCITY,_fluidVelocity);
    archive.read(MULTICOPTER_AIRDEFINITION, _airDefinitionFileName);
    archive.read(MULTICOPTER_WALLEFFECT,_wallEffect);
    archive.read(MULTICOPTER_GROUNDEFFECT, _groundEffect);
    archive.read(MULTICOPTER_OUTPUT, _outputParam);
    archive.read(MULTICOPTER_TIMESTEP, _timeStep);

    if(_airDefinitionFileName=="")return true;

    SimulationManager* simMgr =SimulationManager::instance();
    FluidEnvironment* fluEnv = simMgr->fluidEnvironment();
    bool ret = fluEnv->load(_airDefinitionFileName);
    if( ret == false ){
        UtilityImpl::printErrorMessage(
            fmt::format(" Air Definition File( {} )is not valid", _airDefinitionFileName));
        _airDefinitionFileNameP=_airDefinitionFileName="";
        return true;
    }
    _airDefinitionFileNameP=_airDefinitionFileName;
    UtilityImpl::printMessage("Air Definition File read successfully.");
    return true;
}
