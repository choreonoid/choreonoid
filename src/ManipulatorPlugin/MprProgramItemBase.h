#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_ITEM_BASE_H

#include "MprProgram.h"
#include <cnoid/Item>
#include <typeinfo>
#include <optional>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class BodyItem;
class KinematicBodyItemSet;
class BodyItemKinematicsKit;
class MprPositionStatement;
class MessageOut;

class CNOID_EXPORT MprProgramItemBase : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    virtual ~MprProgramItemBase();

    virtual bool setName(const std::string& name) override;

    BodyItem* targetBodyItem();
    KinematicBodyItemSet* targetBodyItemSet();
    BodyItemKinematicsKit* targetMainKinematicsKit();

    MprProgram* program();
    const MprProgram* program() const;

    bool isStartupProgram() const;
    bool setAsStartupProgram(bool on, bool doNotify = true);

    //! Step index to start execution from.
    //! 0 is the first step. nullopt means unspecified (starts from the beginning).
    std::optional<int> startStep() const;
    void setStartStep(std::optional<int> step);

    //! Returns the base index when displaying step numbers to users.
    //! Override this to return 0 if 0-based indexing is preferred.
    virtual int displayStepIndexBase();

    bool moveTo(MprPositionStatement* statement, MessageOut* mout = nullptr);
    bool moveTo(MprPosition* position, MessageOut* mout = nullptr);
    bool superimposePosition(MprPositionStatement* statement, MessageOut* mout = nullptr);
    bool superimposePosition(MprPosition* position, MessageOut* mout = nullptr);
    void clearSuperimposition();
    bool touchupPosition(MprPositionStatement* statement, MessageOut* mout = nullptr);
    bool touchupPosition(MprPosition* position, MessageOut* mout = nullptr);

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
