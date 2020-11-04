#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H

#include "MprProgram.h"
#include <cnoid/Item>
#include <typeinfo>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class BodyItem;
class LinkKinematicsKit;
class MprPositionStatement;

class CNOID_EXPORT MprProgramItemBase : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    virtual ~MprProgramItemBase();

    virtual bool setName(const std::string& name) override;

    BodyItem* targetBodyItem();

    LinkKinematicsKit* kinematicsKit();

    MprProgram* program();
    const MprProgram* program() const;

    bool isStartupProgram() const;
    bool setAsStartupProgram(bool on, bool doNotify = true);

    bool moveTo(MprPositionStatement* statement);
    bool moveTo(MprPosition* position);
    bool superimposePosition(MprPositionStatement* statement);
    bool superimposePosition(MprPosition* position);
    void clearSuperimposition();
    bool touchupPosition(MprPositionStatement* statement);
    bool touchupPosition(MprPosition* position);

    template<class StatementType>
    static void registerUnreferenceFunction(std::function<bool(StatementType*, MprProgramItemBase*)> unreference){
        registerUnreferenceFunction_(
            typeid(StatementType),
            [unreference](MprStatement* statement, MprProgramItemBase* item){
                return unreference(static_cast<StatementType*>(statement), item); });
    }
    bool resolveProgramDataReferences();

    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    MprProgramItemBase();
    MprProgramItemBase(const MprProgramItemBase& org);
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    static void registerUnreferenceFunction_(
        const std::type_info& type, std::function<bool(MprStatement*, MprProgramItemBase*)> unreference);
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprProgramItemBase> MprProgramItemBasePtr;

}

#endif
