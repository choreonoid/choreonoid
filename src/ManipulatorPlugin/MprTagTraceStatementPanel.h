#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_PANEL_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_TAG_TRACE_STATEMENT_PANEL_H

#include "MprStatementPanel.h"
#include <QGridLayout>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MprTagTraceStatementPanel : public MprStatementPanel
{
public:
    MprTagTraceStatementPanel();
    ~MprTagTraceStatementPanel();

    virtual void setEditingEnabled(bool on) override;
    virtual void onActivated() override;
    virtual void onStatementUpdated() override;

protected:
    void createBaseInterfaces(std::function<void(QGridLayout* grid)> createOptionInterfaces);
    
private:
    class Impl;
    Impl* impl;
};

}

#endif
