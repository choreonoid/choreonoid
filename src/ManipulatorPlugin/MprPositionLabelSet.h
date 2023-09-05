#ifndef CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LABEL_SET_H
#define CNOID_MANIPULATOR_PLUGIN_MPR_POSITION_LABEL_SET_H

#include "MprPosition.h"
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/CheckBox>
#include <QLabel>
#include <QGridLayout>

namespace cnoid {

class BodyItemKinematicsKit;

class MprPositionLabelSet : public Referenced
{
public:
    MprPositionLabelSet(QGridLayout* sharedGrid);
    void showBodyPartLabel(bool on);
    void update(
        BodyItemKinematicsKit* kinematicsKit, MprPosition* position, int& io_row,
        int gridColumnSize, int jointDisplacementColumnSize, bool isJointNameLabelEnabled);
    void detach();
    static void updateCoordinateFrameLabel(QLabel& label, const GeneralId& id, CoordinateFrame* frame);

private:
    void createIkPanel();
    int attachIkPanel(int row, int gridColumnSize);
    void detachIkPanel();
    void updateIkPanel(BodyItemKinematicsKit* kinematicsKit, MprIkPosition* position);
    int attachJointLabels(
        int row, int numJoints, int gridColumnSize,
        int jointDisplacementColumnSize, bool isJointNameLabelEnabled);
    void detachJointLabels();
    void updateJointLabels(BodyItemKinematicsKit* kinematicsKit, MprFkPosition* position, bool isJointNameLabelEnabled);
    void onAutoConfigCheckToggled(bool on);

    QGridLayout* sharedGrid;
    int currentRow;
    MprPosition::PositionType currentPositionType;
    BodyItemKinematicsKitPtr currentKinematicsKit;
    MprIkPositionPtr currentIkPosition;
    
    QLabel bodyPartLabel;

    QWidget ikPanel;
    QLabel xyzLabels[3];
    QLabel rpyLabels[3];
    QLabel coordinateFrameLabels[2];
    QLabel configLabel;
    CheckBox autoConfigCheck;
    
    QLabel jointNameLabels[MprPosition::MaxNumJoints];
    QLabel jointDisplacementLabels[MprPosition::MaxNumJoints];
    int numValidJoints;
};

typedef ref_ptr<MprPositionLabelSet> MprPositionLabelSetPtr;

}

#endif
