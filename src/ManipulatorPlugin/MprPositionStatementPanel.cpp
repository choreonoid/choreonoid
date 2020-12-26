#include "MprPositionStatementPanel.h"
#include "MprPositionStatement.h"
#include "MprProgramItemBase.h"
#include <cnoid/LinkKinematicsKit>
#include <cnoid/MprPosition>
#include <cnoid/MprPositionList>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
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

const char* xyzCaptions[] = { "X:", "Y:", "Z:" };
const char* rpyCaptions[] = { "R:", "P:", "Y:" };

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
    QWidget ikPositionPanel;
    QLabel xyzLabel[3];
    QLabel rpyLabel[3];
    //QLabel coordinateSystemLabel;
    QLabel coordinateFrameLabel[2];
    QLabel configLabel;
    QWidget fkPositionPanel;
    QLabel jointDisplacementLabel[MprFkPosition::MaxNumJoints];

    Impl(MprPositionStatementPanel* self);
    void initializeIkPositionPanel();
    void initializeFkPositionPanel();
    void updatePositionPanel();    
    void updateIkPositionPanel(MprIkPosition* position, LinkKinematicsKit* kinematicsKit);
    void updateFkPositionPanel(MprFkPosition* position);
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

    auto vbox = new QVBoxLayout;
    positionPanel.setLayout(vbox);
    
    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Position :")));
    hbox->addWidget(&positionNameLabel);

    hbox->addSpacing(10);

    moveToButton.setText(_("Move to"));
    moveToButton.sigClicked().connect(
        [self](){
            self->currentProgramItem()->moveTo(
                self->currentStatement<MprPositionStatement>());
        });
    hbox->addWidget(&moveToButton);

    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect(
        [self](){
            self->currentProgramItem()->touchupPosition(
                self->currentStatement<MprPositionStatement>());
        });
    hbox->addWidget(&touchupButton);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    initializeIkPositionPanel();
    vbox->addWidget(&ikPositionPanel);

    initializeFkPositionPanel();
    vbox->addWidget(&fkPositionPanel);

    topVBox->addWidget(&positionPanel);
    topVBox->addStretch();
}


MprPositionStatementPanel::~MprPositionStatementPanel()
{
    delete impl;
}


QWidget* MprPositionStatementPanel::topPanel()
{
    return &impl->topPanel;
}


QWidget* MprPositionStatementPanel::positionPanel()
{
    return &impl->positionPanel;
}


void MprPositionStatementPanel::Impl::initializeIkPositionPanel()
{
    auto vbox = new QVBoxLayout(&ikPositionPanel);
    vbox->setContentsMargins(0, 0, 0, 0);
    
    auto grid = new QGridLayout;
    grid->setContentsMargins(0, 0, 0, 0);

    for(int i=0; i < 3; ++i){
        grid->addWidget(new QLabel(xyzCaptions[i]), 0, i * 2, Qt::AlignCenter);
        grid->addWidget(&xyzLabel[i], 0, i * 2 + 1, Qt::AlignCenter);
        grid->addWidget(new QLabel(rpyCaptions[i]), 1, i * 2, Qt::AlignCenter);
        grid->addWidget(&rpyLabel[i], 1, i * 2 + 1, Qt::AlignCenter);
        grid->setColumnStretch(i * 2, 0);
        grid->setColumnStretch(i * 2 + 1, 1);
    }

    vbox->addLayout(grid);

    grid = new QGridLayout;
    grid->setContentsMargins(0, 0, 0, 0);

    int row = 0;
    /*
    grid->addWidget(new QLabel(_("Coord"), row, 0));
    grid->addWidget(new QLabel(":"), row, 1);
    grid->addWidget(&coordinateSystemLabel, row, 2);
    ++row;
    */
    
    grid->addWidget(new QLabel(_("Base")), row, 0);
    grid->addWidget(new QLabel(":"), row, 1);
    grid->addWidget(&coordinateFrameLabel[0], row, 2);
    ++row;
    
    grid->addWidget(new QLabel(_("Tool")), row, 0);
    grid->addWidget(new QLabel(":"), row, 1);
    grid->addWidget(&coordinateFrameLabel[1], row, 2);
    ++row;

    grid->addWidget(new QLabel(_("Config")), row, 0);
    grid->addWidget(new QLabel(":"), row, 1);
    grid->addWidget(&configLabel, row, 2);
    ++row;

    grid->setColumnStretch(0, 0);
    grid->setColumnStretch(1, 0);
    grid->setColumnStretch(2, 1);
    
    vbox->addLayout(grid);
}


void MprPositionStatementPanel::Impl::initializeFkPositionPanel()
{
    auto grid = new QGridLayout(&fkPositionPanel);
    grid->setContentsMargins(0, 0, 0, 0);
    for(int i=0; i < MprPosition::MaxNumJoints; ++i){
        int row = i / 6;
        int column = i % 6;
        grid->addWidget(&jointDisplacementLabel[i], row, column, Qt::AlignCenter);
    }
}


void MprPositionStatementPanel::setEditingEnabled(bool on)
{
    impl->touchupButton.setEnabled(on);
}


void MprPositionStatementPanel::onStatementUpdated()
{
    impl->updatePositionPanel();
}


void MprPositionStatementPanel::updatePositionPanel()
{
    impl->updatePositionPanel();
}


void MprPositionStatementPanel::Impl::updatePositionPanel()
{
    auto statement = self->currentStatement<MprPositionStatement>();

    auto kinematicsKit = self->currentProgramItem()->kinematicsKit();
    auto position = statement->position();

    ikPositionPanel.setVisible(false);
    fkPositionPanel.setVisible(false);
    
    if(!position){
        positionNameLabel.setText(
            QString(_("%1 ( Not found )")).arg(statement->positionLabel().c_str()));
        positionNameLabel.setStyleSheet(errorStyle);

    } else {
        positionNameLabel.setText(statement->positionLabel().c_str());
        positionNameLabel.setStyleSheet(normalStyle);
    
        if(position->isIK()){
            ikPositionPanel.setVisible(true);
            updateIkPositionPanel(position->ikPosition(), kinematicsKit);
        } else if(position->isFK()){
            fkPositionPanel.setVisible(true);
            updateFkPositionPanel(position->fkPosition());
        }
    }
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
    

void MprPositionStatementPanel::Impl::updateIkPositionPanel
(MprIkPosition* position, LinkKinematicsKit* kinematicsKit)
{
    auto xyz = position->position().translation();
    auto rpy = position->rpy();
    for(int i=0; i < 3; ++i){
        xyzLabel[i].setText(QString::number(xyz[i], 'f', 4));
        rpyLabel[i].setText(QString::number(degree(rpy[i]), 'f', 1));
    }

    updateCoordinateFrameLabel(
        coordinateFrameLabel[0], position->baseFrameId(), position->findBaseFrame(kinematicsKit->baseFrames()),
        kinematicsKit->baseFrames());
    updateCoordinateFrameLabel(
        coordinateFrameLabel[1], position->offsetFrameId(), position->findOffsetFrame(kinematicsKit->offsetFrames()),
        kinematicsKit->offsetFrames());

    int configIndex = position->configuration();
    if(kinematicsKit){
        string configName = kinematicsKit->configurationLabel(configIndex);
        configLabel.setText(format("{0:X} ( {1} )", configIndex, configName).c_str());
    } else {
        configLabel.setText(QString::number(configIndex));
    }
}


void MprPositionStatementPanel::Impl::updateFkPositionPanel(MprFkPosition* position)
{
    const int n = position->numJoints();
    int i = 0;
    while(i < n){
        auto& label = jointDisplacementLabel[i];
        label.setVisible(true);
        double q = position->jointDisplacement(i);
        if(position->checkIfRevoluteJoint(i)){
            label.setText(QString::number(degree(q), 'f', 1));
        } else {
            label.setText(QString::number(q, 'f', 3));
        }
        ++i;
    }
    while(i < MprPosition::MaxNumJoints){
        jointDisplacementLabel[i++].setVisible(false);
    }
}
