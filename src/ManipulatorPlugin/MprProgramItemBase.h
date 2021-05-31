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
    static void registerReferenceResolver(
        std::function<bool(StatementType*, MprProgramItemBase*)> resolve){
        registerReferenceResolver_(
            typeid(StatementType),
            [resolve](MprStatement* statement, MprProgramItemBase* item){
                return resolve(static_cast<StatementType*>(statement), item); });
    }
    bool resolveStatementReferences(MprStatement* statement);
    bool resolveAllReferences();

    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    MprProgramItemBase();
    MprProgramItemBase(const MprProgramItemBase& org);
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void onConnectedToRoot() override;

private:
    static void registerReferenceResolver_(
        const std::type_info& type,
        const std::function<bool(MprStatement*, MprProgramItemBase*)>& resolve);
    
    class Impl;
    Impl* impl;
};

typedef ref_ptr<MprProgramItemBase> MprProgramItemBasePtr;

}

#endif
