#include "MprPositionStatementPanel.h"
#include "MprPositionLabelSet.h"
#include "MprPositionStatement.h"
#include "MprProgramItemBase.h"
#include "MprControllerItemBase.h"
#include <cnoid/MprPosition>
#include <cnoid/MprPositionList>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
#include <cnoid/MessageOut>
#include <QLabel>
#include <QBoxLayout>
#include <QGridLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

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
    QGridLayout* positionLabelGrid;
    int jointDisplacementColumnSize;
    int positionLabelGridColumnSize;
    bool isJointNameLabelEnabled;
    bool needToUpdatePositionLabelGrid;
    vector<MprPositionLabelSetPtr> positionLabelSets;
    int numActivePositionLabelSets;

    Impl(MprPositionStatementPanel* self);
    ~Impl();
    void updateTotalPositionLabelGridColumnSize();
    void updatePositionPanel();
    void updatePositionLabelSet(
        MprPosition* position, BodyItemKinematicsKit* kinematicsKit, int& io_widgetSetIndex, int& io_row);
    MprPositionLabelSet* getOrCreatePositionLabelSet(int index);
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
    updateTotalPositionLabelGridColumnSize();

    positionLabelGrid = new QGridLayout;
    positionLabelGrid->setColumnStretch(positionLabelGridColumnSize, 10);
    needToUpdatePositionLabelGrid = false;

    numActivePositionLabelSets = 0;
    
    positionPanelVBox->addLayout(positionLabelGrid);

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
    impl->needToUpdatePositionLabelGrid = true;
}


void MprPositionStatementPanel::setJointNameLabelEnabled(bool on)
{
    impl->isJointNameLabelEnabled = on;
    impl->needToUpdatePositionLabelGrid = true;
}


void MprPositionStatementPanel::Impl::updateTotalPositionLabelGridColumnSize()
{
    positionLabelGridColumnSize = 1 + jointDisplacementColumnSize * (isJointNameLabelEnabled ? 3 : 1);
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
        positionNameLabel.setStyleSheet("font-weight: bold; color: red");

    } else {
        positionNameLabel.setText(statement->positionLabel().c_str());
        positionNameLabel.setStyleSheet("font-weight: normal");
    }

    if(needToUpdatePositionLabelGrid){
        updateTotalPositionLabelGridColumnSize();
        int i = 0;
        int n = positionLabelGridColumnSize - 1;
        while(i < n){
            positionLabelGrid->setColumnStretch(i++, 0);
        }
        positionLabelGrid->setColumnStretch(i, 10); // last column
        needToUpdatePositionLabelGrid = false;
    }

    int row = 0;
    int labelSetIndex = 0;
    numActivePositionLabelSets = 0;
    
    if(!position->isComposite()){
        if(auto mainKinematicsKit = self->currentMainKinematicsKit()){
            updatePositionLabelSet(position, mainKinematicsKit, labelSetIndex, row);
        }
    } else {
        auto composite = position->compositePosition();
        auto bodyItemSet = self->currentBodyItemSet();
        for(auto& partIndex : composite->findMatchedPositionIndices(bodyItemSet)){
            updatePositionLabelSet(
                composite->position(partIndex),
                bodyItemSet->bodyItemPart(partIndex),
                labelSetIndex,
                row);
        }
    }

    labelSetIndex = 0;
    bool showBodyPartLabels = numActivePositionLabelSets >= 2;
    while(labelSetIndex < numActivePositionLabelSets){
        auto labelSet = positionLabelSets[labelSetIndex];
        labelSet->showBodyPartLabel(showBodyPartLabels);
        ++labelSetIndex;
    }
    while(labelSetIndex < static_cast<int>(positionLabelSets.size())){
        positionLabelSets[labelSetIndex++]->detach();
    }
}


void MprPositionStatementPanel::Impl::updatePositionLabelSet
(MprPosition* position, BodyItemKinematicsKit* kinematicsKit, int& io_labelSetIndex, int& io_row)
{
    auto labelSet = getOrCreatePositionLabelSet(io_labelSetIndex++);
    labelSet->update(
        kinematicsKit, position, io_row,
        positionLabelGridColumnSize, jointDisplacementColumnSize, isJointNameLabelEnabled);
    ++numActivePositionLabelSets;
}


MprPositionLabelSet* MprPositionStatementPanel::Impl::getOrCreatePositionLabelSet(int index)
{
    if(index >= static_cast<int>(positionLabelSets.size())){
        positionLabelSets.resize(index + 1);
    }
    auto& labelSet = positionLabelSets[index];
    if(!labelSet){
        labelSet = new MprPositionLabelSet(positionLabelGrid);
    }
    return labelSet;
}


int MprPositionStatementPanel::numActivePositionParts() const
{
    return impl->numActivePositionLabelSets;
}
