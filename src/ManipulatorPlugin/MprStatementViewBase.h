#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_VIEW_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_VIEW_BASE_H

#include <cnoid/View>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class MprStatement;
class MprProgramViewBase;
class MprProgramItemBase;

class CNOID_EXPORT MprStatementPanel : public QWidget
{
public:
    MprStatementPanel();
    ~MprStatementPanel();
    
    virtual void onActivated() = 0;
    virtual void onDeactivated();
    virtual void onStatementUpdated();

    MprProgramItemBase* currentProgramItem();

    template<class StatementType> StatementType* currentStatement(){
        return dynamic_cast<StatementType*>(getCurrentStatement());
    }

private:
    class Impl;
    Impl* impl;
    MprStatement* getCurrentStatement();
    friend class MprStatementViewBase;
};


class CNOID_EXPORT MprStatementViewBase : public cnoid::View
{
public:
    MprStatementViewBase();
    virtual ~MprStatementViewBase();

    virtual void onActivated() override;
    virtual void onDeactivated() override;

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

protected:
    typedef MprStatementPanel* (*PanelFactoryFunction)();

    template<class StatementType, class StatementPanelType>
    void registerPanelFactory(){
        registerPanelFactory(
            typeid(StatementType),
            []() -> MprStatementPanel* { return new StatementPanelType; });
    }

    virtual MprProgramViewBase* getProgramView() = 0;

private:
    class Impl;
    Impl* impl;

    void registerPanelFactory(const std::type_info& statementType, PanelFactoryFunction factory);
};

}

#endif
