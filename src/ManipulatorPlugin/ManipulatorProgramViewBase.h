#ifndef CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_VIEW_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MANIPULATOR_PROGRAM_VIEW_BASE_H

#include "ManipulatorStatement.h"
#include <cnoid/View>
#include <cnoid/Signal>
#include <cnoid/Referenced>
#include <typeindex>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class ManipulatorProgramItemBase;
class ManipulatorProgram;
class ManipulatorPositionStatement;
class MenuManager;

class CNOID_EXPORT ManipulatorProgramViewBase : public View
{
public:
    ManipulatorProgramViewBase();
    virtual ~ManipulatorProgramViewBase();

    virtual void onDeactivated() override;

    ManipulatorProgramItemBase* currentProgramItem();

    ManipulatorStatement* currentStatement();
    SignalProxy<void(ManipulatorStatement* statement)> sigCurrentStatementChanged();

    void updateStatementTree();

    class CNOID_EXPORT StatementDelegate : public Referenced
    {
    public:
        StatementDelegate();
        ~StatementDelegate();
        static constexpr int SpanToLast = -1;
        virtual int labelSpan(ManipulatorStatement* statement, int column) const;
        virtual QVariant dataOfEditRole(ManipulatorStatement* statement, int column) const;
        virtual void setDataOfEditRole(ManipulatorStatement* statement, int column, const QVariant& value) const;
        virtual QWidget* createEditor(ManipulatorStatement* statement, int column, QWidget* parent) const;
        virtual void setEditorData(ManipulatorStatement* statement, int column, QWidget* editor) const;
        virtual void setStatementData(ManipulatorStatement* statement, int column, QWidget* editor) const;

        class Impl;
        Impl* impl;

    protected:
        // Only calleded from the createEditor function
        QWidget* createDefaultEditor() const;
    };

    template<class StatementType>
    void registerStatementDelegate(StatementDelegate* delegate){
        registerStatementDelegate(typeid(StatementType), delegate);
    }

    template<class StatementType>
    void customizeContextMenu(
        std::function<void(StatementType* statement, MenuManager& menuManager,
                           ManipulatorStatementFunctionDispatcher menuFunction)> func){
        customizeContextMenu_(
            typeid(StatementType),
            [func](ManipulatorStatement* statement, MenuManager& menuManager,
                   ManipulatorStatementFunctionDispatcher menuFunction){
                func(static_cast<StatementType*>(statement), menuManager, menuFunction);
            });
    }

    enum BodySyncMode { NoBodySync, DirectBodySync, TwoStageBodySync };
    void setBodySyncMode(BodySyncMode mode);
    BodySyncMode bodySyncMode() const;

    bool updateBodyPositionWithPositionStatement(
        ManipulatorPositionStatement* ps,
        bool doUpdateCurrentCoordinateFrames = true, bool doNotifyKinematicStateChange = true);

    class Impl;

protected:
    void addStatementButton(QWidget* button, int row);

    enum InsertionType { BeforeTargetPosition, AfterTargetPosition };
    bool insertStatement(
        ManipulatorStatement* statement, int insertionType = AfterTargetPosition);

    virtual bool onCurrentProgramItemChanged(ManipulatorProgramItemBase* item) = 0;
    virtual void onCurrentStatementChanged(ManipulatorStatement* statement);
    // This function is called when the current statemet is changed or clicked.
    virtual void onStatementActivated(ManipulatorStatement* statement);
    virtual void onStatementDoubleClicked(ManipulatorStatement* statement);
    virtual void onOptionMenuRequest(MenuManager& menuManager);
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    void registerStatementDelegate(std::type_index statementType, StatementDelegate* delegate);
    void registerBaseStatementDelegates();

    void customizeContextMenu_(
        const std::type_info& type,
        std::function<void(ManipulatorStatement* statement, MenuManager& menuManager,
                           ManipulatorStatementFunctionDispatcher menuFunction)> func);
    
    Impl* impl;
};

}

#endif
