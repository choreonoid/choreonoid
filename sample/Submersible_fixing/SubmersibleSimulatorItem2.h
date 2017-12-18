/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H
#define CNOID_SAMPLE_SUBMERSIBLE_SIMULATOR_ITEM_H

#ifdef GLOBAL_LINK
  #define GLOBAL
  #define GLOBAL_LINK(v) = (v)
#else
  #define GLOBAL extern
  #define GLOBAL_LINK(v)
#endif

#include <cnoid/SimulatorItem>
#include <cnoid/SubSimulatorItem>
#include <cnoid/BodyItem>
#include "exportdecl.h"
#include <cnoid/ItemList>
#include <vector>

namespace cnoid {

class Light;

class CNOID_EXPORT SubmersibleSimulatorItem2 : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    SubmersibleSimulatorItem2();
    SubmersibleSimulatorItem2(const SubmersibleSimulatorItem2& org);
    ~SubmersibleSimulatorItem2();
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);

    Link* root;
//    Link* getRootLink() const{ return rootLink_; };
//    void setRootLink(Link* rootLink){ rootLink_ = rootLink; };
//    SubmersibleSimulatorItem2* test;

protected:
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    SimulatorItem* simulatorItem;
    bool prevLightButtonState;
    int joystickIntervalCounter;

    void initialize();
    void applyResistanceForce();
    std::vector<Body*> body;
    double max_bbox[3] = {0};
	double min_bbox[3] = {0};
//	Link* rootLink_;
};

typedef ref_ptr<SubmersibleSimulatorItem2> SubmersibleSimulatorItem2Ptr;

}

#endif
