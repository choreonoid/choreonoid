#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H

#include "MprProgram.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class BodyItem;
class LinkKinematicsKit;

class CNOID_EXPORT MprProgramItemBase : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    virtual ~MprProgramItemBase();

    virtual void setName(const std::string& name) override;

    BodyItem* targetBodyItem();

    LinkKinematicsKit* kinematicsKit();

    MprProgram* program();
    const MprProgram* program() const;

    bool isStartupProgram() const;
    bool setAsStartupProgram(bool on, bool doNotify = true);

    virtual void notifyUpdate() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    MprProgramItemBase();
    MprProgramItemBase(const MprProgramItemBase& org);
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprProgramItemBase> MprProgramItemBasePtr;

}

#endif
