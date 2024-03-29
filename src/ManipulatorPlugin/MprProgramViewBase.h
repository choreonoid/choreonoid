#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_VIEW_BASE_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_PROGRAM_VIEW_BASE_H

#include "MprStatement.h"
#include <cnoid/View>
#include <cnoid/Signal>
#include <cnoid/Referenced>
#include <typeindex>
#include "exportdecl.h"

namespace cnoid {

class Archive;
class MprProgramItemBase;
class MprProgram;
class MprPositionStatement;
class MenuManager;
class ToolButton;

class CNOID_EXPORT MprProgramViewBase : public View
{
public:
    MprProgramViewBase();
    virtual ~MprProgramViewBase();

    virtual void onActivated() override;
    virtual void onDeactivated() override;

    MprProgramItemBase* currentProgramItem();
    bool checkCurrentProgramItem() const;

    MprStatement* currentStatement();
    SignalProxy<void(MprStatement* statement)> sigCurrentStatementChanged();

    const std::vector<MprStatementPtr>& selectedStatements();
    SignalProxy<void(std::vector<MprStatementPtr>& statements)> sigSelectedStatementsChanged();

    void updateStatementTree();

    class CNOID_EXPORT StatementDelegate : public Referenced
    {
    public:
        StatementDelegate();
        ~StatementDelegate();
        static constexpr int SpanToLast = -1;
        virtual void activateStatement(MprStatement* statement) const;
        virtual int labelSpan(MprStatement* statement, int column) const;
        virtual QVariant dataOfEditRole(MprStatement* statement, int column) const;
        virtual void setDataOfEditRole(MprStatement* statement, int column, const QVariant& value) const;
        virtual QWidget* createEditor(MprStatement* statement, int column, QWidget* parent) const;
        virtual void setEditorData(MprStatement* statement, int column, QWidget* editor) const;
        virtual void setStatementData(MprStatement* statement, int column, QWidget* editor) const;

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
                           MprStatementFunctionDispatcher menuFunction)> func){
        customizeContextMenu_(
            typeid(StatementType),
            [func](MprStatement* statement, MenuManager& menuManager,
                   MprStatementFunctionDispatcher menuFunction){
                func(static_cast<StatementType*>(statement), menuManager, menuFunction);
            });
    }

    enum BodySyncMode { NoBodySync, DirectBodySync, TwoStageBodySync };
    void setBodySyncMode(BodySyncMode mode);
    BodySyncMode bodySyncMode() const;

    class Impl;

protected:
    virtual void updateMenuItems();
    virtual void updateButtons();
    
    void addEditButton(ToolButton* button, int row);
    void addStatementToMenuBar(QString path, QString name, std::function<void()> func);
    void setStatementInMenuBarEnabled(QString path, bool on);

    enum InsertionType { BeforeTargetPosition, AfterTargetPosition };
    bool insertStatement(
        MprStatement* statement, int insertionType = AfterTargetPosition);

    virtual bool onCurrentProgramItemChanged(MprProgramItemBase* item) = 0;
    virtual void onCurrentStatementChanged(MprStatement* statement);
    // This function is called when the current statemet is changed or clicked.
    virtual void onStatementActivated(MprStatement* statement);
    virtual void onStatementDoubleClicked(MprStatement* statement);

    virtual void onAttachedMenuRequest(MenuManager& menuManager) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    void registerStatementDelegate(std::type_index statementType, StatementDelegate* delegate);
    void registerBaseStatementDelegates();

    void customizeContextMenu_(
        const std::type_info& type,
        std::function<void(MprStatement* statement, MenuManager& menuManager,
                           MprStatementFunctionDispatcher menuFunction)> func);
    
    Impl* impl;
};

}

#endif
