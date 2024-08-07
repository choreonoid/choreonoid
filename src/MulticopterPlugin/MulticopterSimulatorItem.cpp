/**
   @author Japan Atomic Energy Agency
*/

#include "MulticopterPluginHeader.h"
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/Format>

using namespace std;
using namespace cnoid;
using namespace Multicopter;


void MulticopterSimulatorItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<MulticopterSimulatorItem, SubSimulatorItem>(N_("MulticopterSimulatorItem"));
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

    putProperty.min(0.0).decimals(3);
    putProperty(MULTICOPTER_DENSITY,_fluidDensity, changeProperty(_fluidDensity));
    putProperty.decimals(6);
    putProperty(MULTICOPTER_VISCOSITY,_viscosity, changeProperty(_viscosity));
    putProperty.decimals(3);
    putProperty(MULTICOPTER_VELOCITY, str(_fluidVelocity), [&](const string& v){ return toVector3(v, _fluidVelocity); });
    putProperty(MULTICOPTER_AIRDEFINITION, moduleProperty,
                [&](const string& filename){ return setAirDefinitionFile(filename); });

    putProperty(MULTICOPTER_WALLEFFECT, _wallEffect, changeProperty(_wallEffect));
    putProperty(MULTICOPTER_GROUNDEFFECT, _groundEffect, changeProperty(_groundEffect));
    putProperty(MULTICOPTER_OUTPUT, _outputParam, changeProperty(_outputParam));
    putProperty(MULTICOPTER_TIMESTEP, _timeStep, changeProperty(_timeStep));
}

bool
MulticopterSimulatorItem::setAirDefinitionFile(const std::string& filename)
{
    bool result = false;
    SimulationManager* simMgr = SimulationManager::instance();

    if(filename.empty()){
        simMgr->setNewFluidEnvironment();
        UtilityImpl::printMessage("Air definition file was reset.");
        result = true;
    } else {
        FluidEnvironment* fluEnv = simMgr->fluidEnvironment();
        result = fluEnv->load(filename);
        if(!result){
            simMgr->setNewFluidEnvironment();
            _airDefinitionFileName.clear();
            UtilityImpl::printErrorMessage(
                formatC("Air definition file \"{}\" is not valid", _airDefinitionFileName));
        }
    }
    if(result){
        _airDefinitionFileName = filename;
        UtilityImpl::printMessage("Air definition file has been loaded successfully.");
    }
    return result;
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

    if(!_airDefinitionFileName.empty()){
        setAirDefinitionFile(_airDefinitionFileName);
    }
    return true;
}
