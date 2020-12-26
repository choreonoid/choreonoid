#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_PANEL_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_PANEL_H

#include <QWidget>
#include <cnoid/MprStatement>
#include "exportdecl.h"

namespace cnoid {

class MprProgramItemBase;

class CNOID_EXPORT MprStatementPanel : public QWidget
{
public:
    MprStatementPanel();
    ~MprStatementPanel();

    void activate(
        MprProgramItemBase* programItem, MprStatement* statement,
        std::function<void(const std::string& caption)> setCaption);
    void deactivate();

    virtual void setEditingEnabled(bool on);
    virtual void onActivated() = 0;
    virtual void onStatementUpdated();
    virtual void onAdditionalStatementsUpdated(const std::vector<MprStatementPtr>& additionalStatements);
    virtual void onDeactivated();
    
protected:
    MprProgramItemBase* currentProgramItem();

    template<class StatementType> StatementType* currentStatement(){
        return dynamic_cast<StatementType*>(getCurrentStatement());
    }
    MprStatement* currentStatement(){
        return getCurrentStatement();
    }

    void setCaption(const std::string& caption);

private:
    class Impl;
    Impl* impl;

    MprStatement* getCurrentStatement();
};

}

#endif
