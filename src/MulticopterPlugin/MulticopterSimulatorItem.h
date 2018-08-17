/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <cnoid/SubSimulatorItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MulticopterSimulatorItem : public SubSimulatorItem
{
public:

    MulticopterSimulatorItem();

    MulticopterSimulatorItem(const MulticopterSimulatorItem& org);

    ~MulticopterSimulatorItem();

    static void initializeClass(cnoid::ExtensionManager* extMgr);

    virtual bool initializeSimulation(cnoid::SimulatorItem* simulatorItem);

    virtual void finalizeSimulation();

    void setParameterToSimulationManager();

protected:
    virtual cnoid::Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    void onPreDynamicFunction();
    void onMidDynamicFunction();
    void onPostDynamicFunction();

private:
    int _preFuncId, _midFuncId, _postFuncId;

    double _fluidDensity;
    double  _viscosity;
    Vector3 _fluidVelocity;
    std::string _airDefinitionFileName;
    std::string _airDefinitionFileNameP;
    bool _wallEffect;
    bool _groundEffect;
    bool _outputParam;
    double _timeStep;

    SimulatorItem* _curSimItem;
};

typedef cnoid::ref_ptr<MulticopterSimulatorItem> MulticopterSimulatorItemPtr;

}
