#include "MprPositionStatementPanel.h"
#include "MprPositionStatement.h"
#include "MprProgramItemBase.h"
#include "MprControllerItemBase.h"
#include <cnoid/MprPosition>
#include <cnoid/MprPositionList>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/JointPath>
#include <cnoid/DisplayValueFormat>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
#include <cnoid/MessageOut>
#include <QLabel>
#include <QBoxLayout>
#include <QGridLayout>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

const QString normalStyle("font-weight: normal");
const QString errorStyle("font-weight: bold; color: red");
constexpr int PositionPartColumnSize = 10;

class PositionPartWidgetSet : public Referenced
{
public:
    MprPositionStatementPanel::Impl* statementPanelImpl;
    QGridLayout* sharedGrid;
    int currentRow;
    MprPosition::PositionType currentPositionType;
    QLabel bodyPartLabel;

    QWidget ikPanel;
    QLabel xyzLabels[3];
    QLabel rpyLabels[3];
    QLabel coordinateFrameLabels[2];
    QLabel configLabel;
    
    QLabel jointNameLabels[MprPosition::MaxNumJoints];
    QLabel jointDisplacementLabels[MprPosition::MaxNumJoints];
    int numValidJoints;
    
    PositionPartWidgetSet(MprPositionStatementPanel::Impl* statementPanelImpl);
    void createIkPanel();
    void update(BodyItemKinematicsKit* kinematicsKit, MprPosition* position, int& io_row);
    void detach();
    int attachIkPanel(int row);
    void detachIkPanel();
    void updateIkPanel(BodyItemKinematicsKit* kinematicsKit, MprIkPosition* position);
    int attachJointLabels(int row, int numJoints);
    void detachJointLabels();
    void updateJointLabels(BodyItemKinematicsKit* kinematicsKit, MprFkPosition* position);
};

typedef ref_ptr<PositionPartWidgetSet> PositionPartWidgetSetPtr;

}

namespace cnoid {

class MprPositionStatementPanel::Impl
{
public:
    MprPositionStatementPanel* self;
    QWidget topPanel;
    QWidget positionPanel;
    QLabel positionNameLabel;
    PushButton moveToButton;
    PushButton touchupButton;
    QGridLayout* positionPartGrid;
    int jointDisplacementColumnSize;
    int totalPositionPartGridColumnSize;
    bool isJointNameLabelEnabled;
    bool needToUpdatePositionPartGrid;
    vector<PositionPartWidgetSetPtr> positionPartWidgetSets;
    int numActivePositionPartWidgetSets;
    vector<int> redundantPositionIndices;

    Impl(MprPositionStatementPanel* self);
    ~Impl();
    void updateTotalPositionPartGridColumnSize();
    void updatePositionPanel();
    void updatePositionPartWidgetSet(
        MprPosition* position, BodyItemKinematicsKit* kinematicsKit, int& io_widgetSetIndex, int& io_row);
    PositionPartWidgetSet* getOrCreatePositionPartWidgetSet(int index);
};

}


MprPositionStatementPanel::MprPositionStatementPanel()
{
    impl = new Impl(this);
}


MprPositionStatementPanel::Impl::Impl(MprPositionStatementPanel* self)
    : self(self)
{
    auto topHBox = new QHBoxLayout;
    self->setLayout(topHBox);

    auto topVBox = new QVBoxLayout;
    topVBox->setContentsMargins(0, 0, 0, 0);
    topHBox->addLayout(topVBox);
    topHBox->addStretch();

    topVBox->addWidget(&topPanel);

    auto positionPanelVBox = new QVBoxLayout;
    positionPanelVBox->setContentsMargins(0, 0, 0, 0);
    positionPanel.setLayout(positionPanelVBox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Position :")));
    hbox->addWidget(&positionNameLabel);

    hbox->addSpacing(10);

    moveToButton.setText(_("Move to"));
    moveToButton.sigClicked().connect(
        [self](){
            self->currentProgramItem()->moveTo(
                self->currentStatement<MprPositionStatement>(), MessageOut::interactive());
        });
    hbox->addWidget(&moveToButton);

    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect(
        [self](){
            self->currentProgramItem()->touchupPosition(
                self->currentStatement<MprPositionStatement>(), MessageOut::interactive());
        });
    hbox->addWidget(&touchupButton);
    
    hbox->addStretch();
    positionPanelVBox->addLayout(hbox);

    jointDisplacementColumnSize = 6;
    isJointNameLabelEnabled = false;
    updateTotalPositionPartGridColumnSize();

    positionPartGrid = new QGridLayout;
    positionPartGrid->setColumnStretch(totalPositionPartGridColumnSize, 10);
    needToUpdatePositionPartGrid = false;

    numActivePositionPartWidgetSets = 0;
    
    positionPanelVBox->addLayout(positionPartGrid);

    topVBox->addWidget(&positionPanel);
    topVBox->addStretch();
}


MprPositionStatementPanel::~MprPositionStatementPanel()
{
    delete impl;
}


MprPositionStatementPanel::Impl::~Impl()
{

}


QWidget* MprPositionStatementPanel::topPanel()
{
    return &impl->topPanel;
}


QWidget* MprPositionStatementPanel::positionPanel()
{
    return &impl->positionPanel;
}


void MprPositionStatementPanel::setJointDisplacementColumnSize(int n)
{
    impl->jointDisplacementColumnSize = n;
    impl->needToUpdatePositionPartGrid = true;
}


void MprPositionStatementPanel::setJointNameLabelEnabled(bool on)
{
    impl->isJointNameLabelEnabled = on;
    impl->needToUpdatePositionPartGrid = true;
}


void MprPositionStatementPanel::Impl::updateTotalPositionPartGridColumnSize()
{
    totalPositionPartGridColumnSize = 1 + jointDisplacementColumnSize * (isJointNameLabelEnabled ? 3 : 1);
}


void MprPositionStatementPanel::setEditable(bool on)
{
    impl->touchupButton.setEnabled(on);
}


void MprPositionStatementPanel::onStatementUpdated()
{
    impl->updatePositionPanel();
}


void MprPositionStatementPanel::updatePositionPanel()
{
    return impl->updatePositionPanel();
}


void MprPositionStatementPanel::Impl::updatePositionPanel()
{
    auto statement = self->currentStatement<MprPositionStatement>();
    auto position = statement->position();

    if(!position){
        positionNameLabel.setText(
            QString(_("%1 ( Not found )")).arg(statement->positionLabel().c_str()));
        positionNameLabel.setStyleSheet(errorStyle);

    } else {
        positionNameLabel.setText(statement->positionLabel().c_str());
        positionNameLabel.setStyleSheet(normalStyle);
    }

    if(needToUpdatePositionPartGrid){
        updateTotalPositionPartGridColumnSize();
        int i = 0;
        int n = totalPositionPartGridColumnSize - 1;
        while(i < n){
            positionPartGrid->setColumnStretch(i++, 0);
        }
        positionPartGrid->setColumnStretch(i, 10); // last column
        needToUpdatePositionPartGrid = false;
    }

    int row = 0;
    int widgetSetIndex = 0;
    numActivePositionPartWidgetSets = 0;
    
    if(!position->isComposite()){
        if(auto mainKinematicsKit = self->currentMainKinematicsKit()){
            updatePositionPartWidgetSet(position, mainKinematicsKit, widgetSetIndex, row);
        }
    } else {
        auto composite = position->compositePosition();
        auto bodyItemSet = self->currentBodyItemSet();
        for(auto& partIndex : composite->findMatchedPositionIndices(bodyItemSet)){
            updatePositionPartWidgetSet(
                composite->position(partIndex),
                bodyItemSet->bodyItemPart(partIndex),
                widgetSetIndex,
                row);
        }
        redundantPositionIndices = composite->findUnMatchedPositionIndices(bodyItemSet);
    }

    widgetSetIndex = 0;
    bool showBodyPartLabels = numActivePositionPartWidgetSets >= 2;
    while(widgetSetIndex < numActivePositionPartWidgetSets){
        auto panel = positionPartWidgetSets[widgetSetIndex];
        panel->bodyPartLabel.setVisible(showBodyPartLabels);
        ++widgetSetIndex;
    }
    while(widgetSetIndex < static_cast<int>(positionPartWidgetSets.size())){
        positionPartWidgetSets[widgetSetIndex]->detach();
        ++widgetSetIndex;
    }
}


void MprPositionStatementPanel::Impl::updatePositionPartWidgetSet
(MprPosition* position, BodyItemKinematicsKit* kinematicsKit, int& io_widgetSetIndex, int& io_row)
{
    auto partPanel = getOrCreatePositionPartWidgetSet(io_widgetSetIndex++);
    partPanel->update(kinematicsKit, position, io_row);
    ++numActivePositionPartWidgetSets;
}


PositionPartWidgetSet* MprPositionStatementPanel::Impl::getOrCreatePositionPartWidgetSet(int index)
{
    if(index >= static_cast<int>(positionPartWidgetSets.size())){
        positionPartWidgetSets.resize(index + 1);
    }
    auto& widgetSet = positionPartWidgetSets[index];
    if(!widgetSet){
        widgetSet = new PositionPartWidgetSet(this);
    }
    return widgetSet;
}


void MprPositionStatementPanel::updateCoordinateFrameLabel
(QLabel& label, const GeneralId& id, CoordinateFrame* frame, CoordinateFrameList* frames)
{
    bool updated = false;
    if(frames){
        if(frame){
            auto& note = frame->note();
            if(id.isInt() && !note.empty()){
                label.setText(QString("%1 ( %2 )").arg(id.toInt()).arg(note.c_str()));
            } else {
                label.setText(id.label().c_str());
            }
            label.setStyleSheet(normalStyle);
        } else { // Not found
            label.setText(QString("%1 ( Not found )").arg(id.label().c_str()));
            label.setStyleSheet(errorStyle);
        }
        updated = true;
    }
    if(!updated){
        label.setText("---");
        label.setStyleSheet(normalStyle);
    }
}


int MprPositionStatementPanel::numActivePositionParts() const
{
    return impl->numActivePositionPartWidgetSets;
}


PositionPartWidgetSet::PositionPartWidgetSet(MprPositionStatementPanel::Impl* statementPanelImpl)
    : statementPanelImpl(statementPanelImpl),
      sharedGrid(statementPanelImpl->positionPartGrid)
{
    currentRow = -1;
    currentPositionType = MprPosition::InvalidPositionType;
    numValidJoints = 0;
    //bodyPartLabel.setStyleSheet("font-weight: bold");
    createIkPanel();
}


void PositionPartWidgetSet::createIkPanel()
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

    localGrid2->addWidget(new QLabel(_("Config")), row, 0);
    localGrid2->addWidget(new QLabel(":"), row, 1);
    localGrid2->addWidget(&configLabel, row, 2);
    ++row;

    localGrid2->setColumnStretch(0, 0);
    localGrid2->setColumnStretch(1, 0);
    localGrid2->setColumnStretch(2, 1);
    
    vbox->addLayout(localGrid2);
}


void PositionPartWidgetSet::update(BodyItemKinematicsKit* kinematicsKit, MprPosition* position, int& io_row)
{
    bodyPartLabel.setText(QString("[ %1 ]").arg(kinematicsKit->body()->name().c_str()));
    sharedGrid->addWidget(&bodyPartLabel, io_row++, 0, 1, statementPanelImpl->totalPositionPartGridColumnSize);

    if(position->isIK()){
        io_row = attachIkPanel(io_row);
        updateIkPanel(kinematicsKit, position->ikPosition());

    } else if(position->isFK()){
        auto fkPosition = position->fkPosition();
        int numJoints = std::min(fkPosition->numJoints(), kinematicsKit->numJoints());
        io_row = attachJointLabels(io_row, numJoints);
        updateJointLabels(kinematicsKit, fkPosition);
    }
}


void PositionPartWidgetSet::detach()
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


int PositionPartWidgetSet::attachIkPanel(int row)
{
    if(currentRow == row && currentPositionType == MprPosition::IK){
        return row + 1;
    }
    if(currentPositionType == MprPosition::FK){
        detachJointLabels();
    }
    sharedGrid->addWidget(&ikPanel, row, 1, 1, statementPanelImpl->totalPositionPartGridColumnSize);
    ikPanel.show();

    currentRow = row;
    currentPositionType = MprPosition::IK;

    return row + 1;
}


void PositionPartWidgetSet::detachIkPanel()
{
    if(currentPositionType == MprPosition::IK){
        sharedGrid->removeWidget(&ikPanel);
        ikPanel.hide();
        currentRow = -1;
        currentPositionType = MprPosition::InvalidPositionType;
    }
}


void PositionPartWidgetSet::updateIkPanel(BodyItemKinematicsKit* kinematicsKit, MprIkPosition* position)
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

    MprPositionStatementPanel::updateCoordinateFrameLabel(
        coordinateFrameLabels[0], position->baseFrameId(),
        position->findBaseFrame(kinematicsKit->baseFrames()),
        kinematicsKit->baseFrames());

    MprPositionStatementPanel::updateCoordinateFrameLabel(
        coordinateFrameLabels[1], position->offsetFrameId(),
        position->findOffsetFrame(kinematicsKit->offsetFrames()),
        kinematicsKit->offsetFrames());

    int configIndex = position->configuration();
    if(kinematicsKit->configurationHandler()){
        string configName = kinematicsKit->configurationLabel(configIndex);
        configLabel.setText(format("{0:X} ( {1} )", configIndex, configName).c_str());
    } else {
        configLabel.setText(QString::number(configIndex));
    }
}


int PositionPartWidgetSet::attachJointLabels(int row, int numJoints)
{
    if(currentRow == row &&
       currentPositionType == MprPosition::FK &&
       numValidJoints == numJoints){
        return row + (numValidJoints / statementPanelImpl->jointDisplacementColumnSize) + 1;
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
        if(column >= statementPanelImpl->totalPositionPartGridColumnSize){
            ++row;
            column = 1;
        }
        if(statementPanelImpl->isJointNameLabelEnabled){
            auto& nameLabel = jointNameLabels[i];
            sharedGrid->addWidget(&nameLabel, row, column++, Qt::AlignLeft);
            nameLabel.show();
        }
        auto& displacementLabel = jointDisplacementLabels[i];
        sharedGrid->addWidget(&displacementLabel, row, column++, Qt::AlignCenter);
        displacementLabel.show();

        if(statementPanelImpl->isJointNameLabelEnabled){
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


void PositionPartWidgetSet::detachJointLabels()
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


void PositionPartWidgetSet::updateJointLabels(BodyItemKinematicsKit* kinematicsKit, MprFkPosition* position)
{
    for(int i=0; i < numValidJoints; ++i){
        auto& nameLabel = jointNameLabels[i];
        if(statementPanelImpl->isJointNameLabelEnabled){
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
            displacementLabel.setText(QString::number(q, 'f', 3));
        }
        displacementLabel.setVisible(true);
    }
}
