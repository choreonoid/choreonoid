#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_VIEW_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_STATEMENT_VIEW_BASE_H

#include <cnoid/View>
#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class ManipulatorStatement;
class ManipulatorProgramViewBase;
class ManipulatorProgramItemBase;

class CNOID_EXPORT ManipulatorStatementPanel : public QWidget
{
public:
    ManipulatorStatementPanel();
    ~ManipulatorStatementPanel();
    
    virtual void onActivated() = 0;
    virtual void onDeactivated();

    ManipulatorProgramItemBase* currentProgramItem();

    template<class StatementType> StatementType* currentStatement(){
        return dynamic_cast<StatementType*>(getCurrentStatement());
    }

private:
    class Impl;
    Impl* impl;
    ManipulatorStatement* getCurrentStatement();
    friend class ManipulatorStatementViewBase;
};


class CNOID_EXPORT ManipulatorStatementViewBase : public cnoid::View
{
public:
    ManipulatorStatementViewBase();
    virtual ~ManipulatorStatementViewBase();

    virtual void onActivated() override;
    virtual void onDeactivated() override;

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

protected:
    typedef ManipulatorStatementPanel* (*PanelFactoryFunction)();

    template<class StatementType, class StatementPanelType>
    void registerPanelFactory(){
        registerPanelFactory(
            typeid(StatementType),
            []() -> ManipulatorStatementPanel* { return new StatementPanelType; });
    }

    virtual ManipulatorProgramViewBase* getProgramView() = 0;

private:
    class Impl;
    Impl* impl;

    void registerPanelFactory(const std::type_info& statementType, PanelFactoryFunction factory);
};

}

#endif
