/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_GL_VISION_SIMULATOR_ITEM_H_INCLUDED
#define CNOID_BODYPLUGIN_GL_VISION_SIMULATOR_ITEM_H_INCLUDED

#include "SubSimulatorItem.h"
#include "exportdecl.h"

namespace cnoid {

class GLVisionSimulatorItemImpl;

class CNOID_EXPORT GLVisionSimulatorItem : public SubSimulatorItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    GLVisionSimulatorItem();
    GLVisionSimulatorItem(const GLVisionSimulatorItem& org);
    ~GLVisionSimulatorItem();
        
    virtual bool initializeSimulation(SimulatorItem* simulatorItem);
    virtual void finalizeSimulation();

protected:
    virtual ItemPtr doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    GLVisionSimulatorItemImpl* impl;
};
}

#endif
