#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_PANEL_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_STATEMENT_PANEL_H

#include "MprStatementPanel.h"
#include <QLabel>
#include "exportdecl.h"

namespace cnoid {

class GeneralId;
class CoordinateFrame;
class CoordinateFrameList;

class CNOID_EXPORT MprPositionStatementPanel : public MprStatementPanel
{
public:
    MprPositionStatementPanel();
    ~MprPositionStatementPanel();

    virtual void setEditingEnabled(bool on) override;
    virtual void onStatementUpdated() override;

    static void updateCoordinateFrameLabel(
        QLabel& label, const GeneralId& id, CoordinateFrame* frame, CoordinateFrameList* frames);

protected:
    QWidget* topPanel();
    QWidget* positionPanel();
    void updatePositionPanel();
    
private:
    class Impl;
    Impl* impl;
};

}

#endif
