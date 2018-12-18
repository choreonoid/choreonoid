/**
   @author Japan Atomic Energy Agency
*/

#pragma once

namespace cnoid {
class MulticopterSimulatorItem;
}

namespace Multicopter {

class FluidEnvironment;
class FluidValue;
class MulticopterMonitorView;

class SimulationManager
{
friend class cnoid::MulticopterSimulatorItem;
public:

    static SimulationManager* instance();

    bool initialize(cnoid::ExtensionManager* extMgr);

    bool isInitialized() const;

    FluidEnvironment* fluidEnvironment();
    FluidEnvironment* fluidEnvironmentSim();

    bool initializeSimulation(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    void finalizeSimulation(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    void preDynamicFunction(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    void midDynamicFunction(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    void postDynamicFunction(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    bool isLinkForceSyncDumpEnabled() const;

    void setLinkForceSyncDumpEnabled(bool flg);

    void setDegreeNumber(int degreeNumber);
    int getDegreeNumber() const;

    MulticopterMonitorView* multicopterMonitorView(){
        return _multicopterMonitorView;
    }

    void setFluidDensity(double fluidDensity);
    void setViscosity(double viscosity);
    void setFluidVelocity(const cnoid::Vector3 &fluidVelocity);
    void setWallEffect(bool wallEffect);
    void setGroundEffect(bool groundEffect);
    bool isLogEnabled() const;
    void setLogEnabled(bool flg);
    void setLogInterval(double sec);
    void setNewFluidEnvironment();

    void debugList();
    void debugMapLinkAttribute(std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> >);
    void debugRotor();


protected:

    class RotorOutValue{
    public:
        std::string linkName;
        std::string rotorName;
        Eigen::Vector3d position;
        Eigen::Vector3d direction;
        double magnitude;
        double torque;
        double wallEffectForce;
        double groundEffectForce;
    };

    class FluidOutValue{
    public:
        bool logMode;
        std::string linkName;
        std::string bodyName;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d accelerationRaw;
        Eigen::Vector3d acceleration;
        Eigen::Vector3d buoyancyForce;
        Eigen::Vector3d addMassForce;
        Eigen::Vector3d addInertiaTorque;
        Eigen::Vector3d surfaceForce;
        Eigen::Vector3d rotationalVelocity;
        Eigen::Vector3d rotationalAccelerationRaw;
        Eigen::Vector3d rotationalAcceleration;
    };

    SimulationManager();

    ~SimulationManager();

    void updateBodyLinkMap(const std::list<cnoid::Body*>& bodyList);

    void clearBodyLinkMap();

    void updateLinkPolygon(const std::vector<cnoid::Link*>& linkAry);

    void clearLinkPolygon();

    void updateLinkState(double time);

    void clearLinkState();

    std::list<cnoid::Body*> simulationTargetBodies(cnoid::SimulatorItem* simItem);

    std::list<cnoid::Body*> fluidDynamicsTargetBodies(cnoid::SimulatorItem* simItem);

    LinkAttribute linkAttribute(const cnoid::Link* link) const;

    const std::vector<LinkTriangleAttribute>& linkPolygon(const cnoid::Link* link) const;

    void logProc(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem);

    void onSimulationStartEnd(bool flg, double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem);

    void onSimulationStep(double time, cnoid::SimulatorItem* simItem, cnoid::SubSimulatorItem* subSimItem);

    void onBodyCreate(const QUuid&, const cnoid::Body* body, const std::vector<cnoid::Link*>& linkAry, const std::vector<cnoid::Device*>& devAry);

    void onBodyDelete(const QUuid&, const cnoid::Body* body, const std::vector<cnoid::Link*>& linkAry, const std::vector<cnoid::Device*>& devAry);

    void calculateSurfaceCuttoffCoefficient(std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute>>& linkBodyMap,
                                            std::map<cnoid::Link*, std::vector<LinkTriangleAttribute>>& linkPolygonMap);
    

    void calcCuttoffCoef (const FFCalc::CutoffCoef& cutoffCalc, const FFCalc::GaussTriangle3d& tri, const std::vector<FFCalc::GaussTriangle3d>& trgTriAry,double coefs[]);

    std::unique_ptr<FFCalc::LinkForce>
    midDynamicFunctionLink(cnoid::SimulatorItem* simItem, cnoid::MulticopterSimulatorItem* fluidSimItem, cnoid::Link& link, const FFCalc::LinkState& linkState,std::map<int,std::tuple<double,cnoid::Vector3>> effectMap,bool calFlag);

    std::list<RotorDevice*> targetRotorDevices() const;
    std::list<RotorDevice*> targetRotorDevices(cnoid::Link* link) const;

private:
    static SimulationManager* _inst;
    
    bool _isInitialized;

    Eigen::Vector3d _gravity;
    
    std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> > _nonSimLinkBodyMap;

    std::map<cnoid::Body*,std::vector<cnoid::Link*>> _bodyLinkMap;
    std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> >_linkBodyMap;
    std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> >_fluidLinkBodyMap;
    std::map<cnoid::Link*, std::tuple<cnoid::Body*, LinkAttribute> > _effectLinkBodyMap;
    std::map<cnoid::Link*, std::vector<LinkTriangleAttribute>>_linkPolygonMap;
    std::map<const cnoid::Link*, FFCalc::LinkStatePtr> _linkStateMap;

    std::list<RotorOutValue> _rotorOutValAry;
    std::list<FluidOutValue> _linkOutValAry;
    
    double _fluidDensity;
    double  _viscosity;
    cnoid::Vector3 _fluidVelocity;
    FluidEnvironment* _fluEnv;
    bool _wallEffect;
    bool _groundEffect;
    bool _logFlg;
    double _logIntrv;

    double _fluidDensitySim;
    double  _viscositySim;
    cnoid::Vector3 _fluidVelocitySim;
    FluidEnvironment* _fluEnvSim;
    bool _wallEffectSim;
    bool _groundEffectSim;
    bool _logFlgSim;
    double _logIntrvSim;
    FluidEnvironment::FluidValue _fluEnvAllSim;
    double _nextLogTime;

    int _effectLinkBodyMapSize;
    int _degree;

    MulticopterMonitorView* _multicopterMonitorView;
    cnoid::AISTCollisionDetectorPtr _collisionDetector;

    QUuid _simStartEndEvId;
    QUuid _simStepEvId;
    QUuid _bodyCreateEvId;
    QUuid _bodyDeleteEvId;

    bool _enableLinkForceDump;

};
}
