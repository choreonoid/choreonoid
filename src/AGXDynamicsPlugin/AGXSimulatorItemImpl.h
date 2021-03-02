#ifndef CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H
#define CNOID_AGXDYNAMICS_PLUGIN_AGX_SIMULATOR_ITEM_IMPL_H

#include <cnoid/SimulatorItem>
#include <cnoid/Selection>
#include "AGXScene.h"
#include "AGXBody.h"
#include <agxCollide/Contacts.h>

namespace cnoid {

class ContactMaterial;
class AGXSimulatorItem;

class AGXSimulatorItemImpl : public Referenced
{
public:
    AGXSimulatorItemImpl(AGXSimulatorItem* self);
    AGXSimulatorItemImpl(AGXSimulatorItem* self, const AGXSimulatorItemImpl& org);
    ~AGXSimulatorItemImpl();

    void initialize();                          // call from defualt constructor
    void doPutProperties(PutPropertyFunction& putProperty);     // add parameters to property panel
    bool store(Archive& archive);               // save simulation parameter to cnoid file
    bool restore(const Archive& archive);       // store simulation parameter from cnoid file

    // Function of create, step simulation
    SimulationBody* createSimulationBody(Body* orgBody);
    bool initializeSimulation(const std::vector<SimulationBody*>& simBodies);
    void createAGXMaterialTable();
    void createAGXContactMaterial(int id1, int id2, ContactMaterial* mat);
    void setAdditionalAGXMaterialParam();
    bool stepSimulation(const std::vector<SimulationBody*>& activeSimBodies);
    void updateLinkContactPoints();
    void updateLinkContactPoints(agxCollide::GeometryContact* contact, Link* link, double direction);
    void stopSimulation();
    void pauseSimulation();
    void restartSimulation();

    void setGravity(const Vector3& g);
    Vector3 getGravity() const;
    void setNumThreads(unsigned int num);
    void setEnableContactReduction(bool bOn);
    void setContactReductionBinResolution(int r);
    void setContactReductionThreshhold(int t);
    void setEnableContactWarmstarting(bool bOn);
    void setEnableAMOR(bool bOn);
    bool saveSimulationToAGXFile();

private:
    ref_ptr<AGXSimulatorItem> self;
    AGXSceneRef agxScene;
    bool doUpdateLinkContactPoints;
    Vector3 m_p_gravity;
    int     m_p_numThreads;
    bool    m_p_enableContactReduction;
    int     m_p_contactReductionBinResolution;
    int     m_p_contactReductionThreshhold;
    bool    m_p_enableContactWarmstarting;
    bool    m_p_enableAMOR;
    bool    m_p_enableAutoSleep;
    bool    m_p_saveToAGXFileOnStart;
    Selection m_p_debugMessageOnConsoleType;
    AGXScene* getAGXScene();
};

}
#endif
