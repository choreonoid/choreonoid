#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_ITEM_BASE_H

#include "ManipulatorProgram.h"
#include <cnoid/Item>
#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class BodyItem;
class BodyManipulatorManager;

class CNOID_EXPORT ManipulatorProgramItemBase : public Item
{
public:
    virtual ~ManipulatorProgramItemBase();

    virtual void setName(const std::string& name) override;

    BodyItem* targetBodyItem();

    BodyManipulatorManager* manipulatorManager();
    SignalProxy<void(BodyManipulatorManager* manager)> sigManipulatorChanged();

    ManipulatorProgram* program();
    const ManipulatorProgram* program() const;

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
