#include "MprPositionLabelSet.h"
#include <cnoid/DisplayValueFormat>
#include <cnoid/MathUtil>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;


MprPositionLabelSet::MprPositionLabelSet(QGridLayout* sharedGrid)
    : sharedGrid(sharedGrid)
{
    currentRow = -1;
    currentPositionType = MprPosition::InvalidPositionType;
    numValidJoints = 0;
    //bodyPartLabel.setStyleSheet("font-weight: bold");
    createIkPanel();
}


void MprPositionLabelSet::showBodyPartLabel(bool on)
{
    bodyPartLabel.setVisible(on);
}


void MprPositionLabelSet::createIkPanel()
{
    auto vbox = new QVBoxLayout(&ikPanel);
    vbox->setContentsMargins(0, 0, 0, 0);
    
    auto localGrid1 = new QGridLayout;
    localGrid1->setContentsMargins(0, 0, 0, 0);

    static const char* xyzCaptions[] = { "X:", "Y:", "Z:" };
    static const char* rpyCaptions[] = { "R:", "P:", "Y:" };

    for(int i=0; i < 3; ++i){
        localGrid1->addWidget(new QLabel(xyzCaptions[i]), 0, i * 2, Qt::AlignCenter);
        auto xyzLabel = &xyzLabels[i];
        localGrid1->addWidget(xyzLabel, 0, i * 2 + 1, Qt::AlignCenter);
        localGrid1->addWidget(new QLabel(rpyCaptions[i]), 1, i * 2, Qt::AlignCenter);
        auto rpyLabel = &rpyLabels[i];
        localGrid1->addWidget(rpyLabel, 1, i * 2 + 1, Qt::AlignCenter);
        localGrid1->setColumnStretch(i * 2, 0);
        localGrid1->setColumnStretch(i * 2 + 1, 1);
    }
    vbox->addLayout(localGrid1);

    auto localGrid2 = new QGridLayout;
    localGrid2->setContentsMargins(0, 0, 0, 0);
    int row = 0;
    
    localGrid2->addWidget(new QLabel(_("Base")), row, 0);
    localGrid2->addWidget(new QLabel(":"), row, 1);
    localGrid2->addWidget(&coordinateFrameLabels[0], row, 2);
    ++row;
    
    localGrid2->addWidget(new QLabel(_("Tool")), row, 0);
    localGrid2->addWidget(new QLabel(":"), row, 1);
    localGrid2->addWidget(&coordinateFrameLabels[1], row, 2);
    ++row;

    autoConfigCheck.setText(_("Auto"));
    autoConfigCheck.sigToggled().connect([this](bool on) {
        onAutoConfigCheckToggled(on); 
    });
    localGrid2->addWidget(new QLabel(_("Config")), row, 0);
    localGrid2->addWidget(new QLabel(":"), row, 1);
    localGrid2->addWidget(&configLabel, row, 2);
    localGrid2->addWidget(&autoConfigCheck, row, 3);
    ++row;

    localGrid2->setColumnStretch(0, 0);
    localGrid2->setColumnStretch(1, 0);
    localGrid2->setColumnStretch(2, 1);
    
    vbox->addLayout(localGrid2);
}


void MprPositionLabelSet::update
(BodyItemKinematicsKit* kinematicsKit, MprPosition* position, int& io_row,
 int gridColumnSize, int jointDisplacementColumnSize, bool isJointNameLabelEnabled)
{
    bodyPartLabel.setText(QString("[ %1 ]").arg(kinematicsKit->body()->name().c_str()));
    sharedGrid->addWidget(&bodyPartLabel, io_row++, 0, 1, gridColumnSize);

    if(position->isIK()){
        io_row = attachIkPanel(io_row, gridColumnSize);
        updateIkPanel(kinematicsKit, position->ikPosition());

    } else if(position->isFK()){
        auto fkPosition = position->fkPosition();
        int numJoints = std::min(fkPosition->numJoints(), kinematicsKit->numJoints());
        io_row = attachJointLabels(io_row, numJoints, gridColumnSize, jointDisplacementColumnSize, isJointNameLabelEnabled);
        updateJointLabels(kinematicsKit, fkPosition, isJointNameLabelEnabled);
    }
}


void MprPositionLabelSet::detach()
{
    sharedGrid->removeWidget(&bodyPartLabel);
    bodyPartLabel.hide();
    
    if(currentPositionType == MprPosition::IK){
        detachIkPanel();
    } else if(currentPositionType == MprPosition::FK){
        detachJointLabels();
    }
    currentPositionType = MprPosition::InvalidPositionType;
}


int MprPositionLabelSet::attachIkPanel(int row, int gridColumnSize)
{
    if(currentRow == row && currentPositionType == MprPosition::IK){
        return row + 1;
    }
    if(currentPositionType == MprPosition::FK){
        detachJointLabels();
    }
    sharedGrid->addWidget(&ikPanel, row, 1, 1, gridColumnSize);
    ikPanel.show();

    currentRow = row;
    currentPositionType = MprPosition::IK;

    return row + 1;
}


void MprPositionLabelSet::detachIkPanel()
{
    if(currentPositionType == MprPosition::IK){
        sharedGrid->removeWidget(&ikPanel);
        ikPanel.hide();
        currentRow = -1;
        currentPositionType = MprPosition::InvalidPositionType;
        currentKinematicsKit.reset();
        currentIkPosition.reset();
    }
}


void MprPositionLabelSet::updateIkPanel(BodyItemKinematicsKit* kinematicsKit, MprIkPosition* position)
{
    auto xyz = position->position().translation();
    if(DisplayValueFormat::instance()->isMillimeter()){
        for(int i=0; i < 3; ++i){
            xyzLabels[i].setText(QString::number(xyz[i] * 1000.0, 'f', 3));
        }
    } else {
        for(int i=0; i < 3; ++i){
            xyzLabels[i].setText(QString::number(xyz[i], 'f', 4));
        }
    }
    auto rpy = position->rpy();
    for(int i=0; i < 3; ++i){
        rpyLabels[i].setText(QString::number(degree(rpy[i]), 'f', 1));
    }

    updateCoordinateFrameLabel(
        coordinateFrameLabels[0], position->baseFrameId(),
        position->findBaseFrame(kinematicsKit->baseFrames()));

    updateCoordinateFrameLabel(
        coordinateFrameLabels[1], position->offsetFrameId(),
        position->findOffsetFrame(kinematicsKit->offsetFrames()));

    int configIndex = position->configuration();
    if(kinematicsKit->configurationHandler()){
        string configName = kinematicsKit->configurationLabel(configIndex);
        configLabel.setText(format("{0:X} ( {1} )", configIndex, configName).c_str());
        currentKinematicsKit = kinematicsKit;
        currentIkPosition = position;

        autoConfigCheck.setChecked(0 == configIndex);
        autoConfigCheck.setEnabled(true);
    } else {
        configLabel.setText(QString::number(configIndex));
        autoConfigCheck.setEnabled(false);
    }
}


void MprPositionLabelSet::updateCoordinateFrameLabel
(QLabel& label, const GeneralId& id, CoordinateFrame* frame)
{
    bool updated = false;
    
    if(frame){
        auto& note = frame->note();
        if(id.isInt() && !note.empty()){
            label.setText(QString("%1 ( %2 )").arg(id.toInt()).arg(note.c_str()));
        } else {
            label.setText(id.label().c_str());
        }
        label.setStyleSheet("font-weight: normal");
    } else { // Not found
        label.setText(QString("%1 ( Not found )").arg(id.label().c_str()));
        label.setStyleSheet("font-weight: bold; color: red");
    }
    
    updated = true;
    
    if(!updated){
        label.setText("---");
        label.setStyleSheet("font-weight: normal");
    }
}


int MprPositionLabelSet::attachJointLabels
(int row, int numJoints, int gridColumnSize, int jointDisplacementColumnSize, bool isJointNameLabelEnabled)
{
    if(currentRow == row &&
       currentPositionType == MprPosition::FK &&
       numValidJoints == numJoints){
        return row + (numValidJoints / jointDisplacementColumnSize) + 1;
    }

    if(currentPositionType == MprPosition::IK){
        detachIkPanel();
    }

    currentRow = row;
    currentPositionType = MprPosition::FK;
    numValidJoints = numJoints;

    int i = 0;
    int column = 1;
    while(i < numJoints){
        if(column >= gridColumnSize){
            ++row;
            column = 1;
        }
        if(isJointNameLabelEnabled){
            auto& nameLabel = jointNameLabels[i];
            sharedGrid->addWidget(&nameLabel, row, column++, Qt::AlignLeft);
            nameLabel.show();
        }
        auto& displacementLabel = jointDisplacementLabels[i];
        sharedGrid->addWidget(&displacementLabel, row, column++, Qt::AlignCenter);
        displacementLabel.show();

        if(isJointNameLabelEnabled){
            ++column; // space between joints
        }
        
        ++i;
    }
    while(i < MprPosition::MaxNumJoints){
        auto& nameLabel = jointNameLabels[i];
        sharedGrid->removeWidget(&nameLabel);
        nameLabel.hide();
        auto& displacementLabel = jointDisplacementLabels[i];
        sharedGrid->removeWidget(&displacementLabel);
        displacementLabel.hide();
        ++i;
    }

    return row + 1;
}


void MprPositionLabelSet::detachJointLabels()
{
    if(currentPositionType == MprPosition::FK){
        for(int i=0; i < MprPosition::MaxNumJoints; ++i){
            auto& nameLabel = jointNameLabels[i];
            sharedGrid->removeWidget(&nameLabel);
            nameLabel.hide();
            auto& displacementLabel = jointDisplacementLabels[i];
            sharedGrid->removeWidget(&displacementLabel);
            displacementLabel.hide();
        }
        currentRow = -1;
        currentPositionType = MprPosition::InvalidPositionType;
        numValidJoints = 0;
    }
}


void MprPositionLabelSet::updateJointLabels
(BodyItemKinematicsKit* kinematicsKit, MprFkPosition* position, bool isJointNameLabelEnabled)
{
    const double lengthRatio = DisplayValueFormat::instance()->isMillimeter() ? 1000.0 : 1.0;
    
    for(int i=0; i < numValidJoints; ++i){
        auto& nameLabel = jointNameLabels[i];
        if(isJointNameLabelEnabled){
            nameLabel.setText(QString("%1:").arg(kinematicsKit->joint(i)->jointName().c_str()));
            nameLabel.show();
        } else {
            nameLabel.hide();
        }
        auto& displacementLabel = jointDisplacementLabels[i];
        double q = position->jointDisplacement(i);
        if(position->checkIfRevoluteJoint(i)){
            displacementLabel.setText(QString::number(degree(q), 'f', 1));
        } else {
            displacementLabel.setText(QString::number(lengthRatio * q, 'f', 3));
        }
        displacementLabel.setVisible(true);
    }
}

void MprPositionLabelSet::onAutoConfigCheckToggled(bool on)
{
    if(currentIkPosition){
        int configIndex = currentIkPosition->configuration();
        if(currentKinematicsKit) {
            if(on) {
                configIndex = 0;
            } else {
                configIndex = currentKinematicsKit->currentConfigurationType();
            }
            currentIkPosition->setConfiguration(configIndex);
            string configName = currentKinematicsKit->configurationLabel(configIndex);
            configLabel.setText(format("{0:X} ( {1} )", configIndex, configName).c_str());
        } else {
            configLabel.setText(QString::number(configIndex));
        }
    } else {
        configLabel.setText(QString("( Not found )"));
    }
}
