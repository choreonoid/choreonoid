#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_PANEL_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_STATEMENT_PANEL_H

#include <QWidget>
#include "exportdecl.h"

namespace cnoid {

class MprStatement;
class MprProgramViewBase;
class MprProgramItemBase;

class CNOID_EXPORT MprStatementPanel : public QWidget
{
public:
    MprStatementPanel();
    ~MprStatementPanel();
    
    virtual void setEditingEnabled(bool on);
    virtual void onActivated() = 0;
    virtual void onDeactivated();
    virtual void onStatementUpdated();

    MprProgramItemBase* currentProgramItem();

    template<class StatementType> StatementType* currentStatement(){
        return dynamic_cast<StatementType*>(getCurrentStatement());
    }
    MprStatement* currentStatement(){
        return getCurrentStatement();
    }

private:
    class Impl;
    Impl* impl;

    void activate(MprProgramItemBase* programItem, MprStatement* statement);
    void deactivate();
    MprStatement* getCurrentStatement();
    
    friend class MprStatementViewBase;
};

}

#endif
