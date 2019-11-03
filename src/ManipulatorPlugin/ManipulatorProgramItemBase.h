#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_ITEM_BASE_H

#include "ManipulatorProgram.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class BodyItem;
class LinkKinematicsKit;

class CNOID_EXPORT ManipulatorProgramItemBase : public Item
{
public:
    virtual ~ManipulatorProgramItemBase();

    virtual void setName(const std::string& name) override;

    BodyItem* targetBodyItem();

    LinkKinematicsKit* kinematicsKit();

    ManipulatorProgram* program();
    const ManipulatorProgram* program() const;

    virtual void notifyUpdate() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    ManipulatorProgramItemBase();
    ManipulatorProgramItemBase(const ManipulatorProgramItemBase& org);
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<ManipulatorProgramItemBase> ManipulatorProgramItemBasePtr;

}

#endif
