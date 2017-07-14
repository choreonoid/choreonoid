#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SCENE_H

#include <agxSDK/Simulation.h>

namespace cnoid{

struct AGXSimulationDesc
{
    AGXSimulationDesc(){}
    agx::UInt8 binResolution;
    agx::UInt  threshhold;
    agx::Vec3  gravity;
    agx::Real  timeStep;
};

struct AGXMaterialDesc
{
    AGXMaterialDesc(){
        name = "defualt";
        density = 1000;
        youngsModulus = 4.0E8;
        poissonRatio = 0.3;
        viscosity = 0.5;
        damping = 0.075;
        roughness = 0.416667;
        surfaceViscosity = 5E-09;
        adhesionForce = 0.0;
        adhesivOverlap = 0.0;
    }
    agx::String name;
    agx::Real density;              // [kg/m^3]
    agx::Real youngsModulus;        // stiffness[Pa]
    agx::Real poissonRatio;

    // Below are overried when ContactMaterials are used.
    agx::Real viscosity;            // relation to restitution. compliace.
    agx::Real damping;              // relax time of penetration
    agx::Real roughness;            // relation to friction
    agx::Real surfaceViscosity;     // wetness
    agx::Real adhesionForce;        // attracive force[N]
    agx::Real adhesivOverlap;       // range[m]
};

enum AGXFrictionModelType
{
    DEFAULT,
    BOX,
    SCALE_BOX,
    ITERATIVE_PROJECTED_CONE
};

struct AGXContactMaterialDesc
{
    AGXContactMaterialDesc(){
        nameA = "defaultA";
        nameB = "defaultB";
        youngsModulus = 2.0E8;
        restitution = 0.5;
        damping = 0.075;
        friction = 0.416667;
        surfaceViscosity = 1.0E-8;
        adhesionForce = 0.0;
        adhesivOverlap = 0.0;
        frictionDirection = agx::ContactMaterial::FrictionDirection::BOTH_PRIMARY_AND_SECONDARY;
        frictionModelType = AGXFrictionModelType::DEFAULT;
        solveType =  agx::FrictionModel::SolveType::SPLIT;
    }
    agx::String nameA;              // material name
    agx::String nameB;              // material name
    agx::Real youngsModulus;        // stiffness[Pa], (m1.ym * m2.ym)/(m1.ym + m2.ym)
    agx::Real restitution;          // 0:perfectly inelastic collision, 1:perfectly elastic collision, sqrt((1-m1.visco) * (1-m2.vico))
    agx::Real damping;              // relax time of penetration(loop count?)
    agx::Real friction;             // sqrt(m1.rough * m2.rough)
    agx::Real surfaceViscosity;     // m1.svisco + m2.svisco
    agx::Real adhesionForce;        // attracive force[N], m1.ad + m2.ad
    agx::Real adhesivOverlap;       // 
    agx::ContactMaterial::FrictionDirection frictionDirection;
    AGXFrictionModelType frictionModelType;
    agx::FrictionModel::SolveType solveType;
};

class AGXScene : public agx::Referenced
{
public:
    AGXScene();
    static AGXScene* create();
    void clearAGXScene();
    void stepAGXSimulation();
    bool saveSceneToAGXFile();
    void setCollisionPair(const unsigned& id1, const unsigned& id2, bool bOn);
    void setCollisionPair(const agx::Name& name1, const agx::Name& name2, bool bOn);

    agxSDK::SimulationRef createAGXSimulation(const AGXSimulationDesc& desc);
    agxSDK::SimulationRef getAGXSimulation();
    void createAGXMaterial(const AGXMaterialDesc& desc);
    agx::MaterialRef getAGXMaterial(const agx::String& materialName);
    void createAGXContactMaterial(const AGXContactMaterialDesc& desc);
private:
    agxSDK::SimulationRef agxSimulation;


};
typedef agx::ref_ptr<AGXScene> AGXSceneRef;
}
#endif