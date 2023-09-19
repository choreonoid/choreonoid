#include "MprTagTraceStatementPanel.h"
#include "MprPositionLabelSet.h"
#include "MprTagTraceStatement.h"
#include "MprProgramItemBase.h"
#include <cnoid/WorldItem>
#include <cnoid/KinematicBodyItemSet>
#include <cnoid/BodyItemKinematicsKit>
#include <cnoid/PositionTagGroupItem>
#include <cnoid/ItemList>
#include <cnoid/CoordinateFrameList>
#include <cnoid/DisplayValueFormat>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
#include <cnoid/ComboBox>
#include <QLabel>
#include <QBoxLayout>
#include <fmt/format.h>
#include <set>
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

class MprTagTraceStatementPanel::Impl
{
public:
    MprTagTraceStatementPanel* self;
    bool isSubBodyPositionEnabled;
    bool isEditable;
    QGridLayout* grid;
    ComboBox tagGroupCombo;
    QLabel tagGroupLabel;
    PushButton touchupButton;
    QLabel xyzLabel[3];
    QLabel rpyLabel[3];
    QLabel baseFrameLabel;
    QLabel offsetFrameLabel;

    QGridLayout* positionLabelGrid;
    int jointDisplacementColumnSize;
    int positionLabelGridColumnSize;
    vector<MprPositionLabelSetPtr> positionLabelSets;

    Impl(MprTagTraceStatementPanel* self);
    void createBaseInterfaces(
        const std::function<void(QGridLayout* grid)>& createAdditionalInterfaces);
    void updateBaseInterfaces();
    MprPositionLabelSet* getOrCreatePositionLabelSet(int index);
    void updateTagGroupCombo();
    void onTagGroupComboActivated(int comboIndex);
    void touchupPositionAndFrames();
};

}


MprTagTraceStatementPanel::MprTagTraceStatementPanel()
{
    impl = new Impl(this);
}


MprTagTraceStatementPanel::Impl::Impl(MprTagTraceStatementPanel* self)
    : self(self)
{
    isEditable = true;
    isSubBodyPositionEnabled = false;
}


void MprTagTraceStatementPanel::setSubBodyPositionEnabled(bool on)
{
    impl->isSubBodyPositionEnabled = on;
}


void MprTagTraceStatementPanel::createBaseInterfaces
(std::function<void(QGridLayout* grid)> createAdditionalInterfaces)
{
    impl->createBaseInterfaces(createAdditionalInterfaces);
}


void MprTagTraceStatementPanel::Impl::createBaseInterfaces
(const std::function<void(QGridLayout* grid)>& createAdditionalInterfaces)
{
    auto vbox = new QVBoxLayout;

    grid = new QGridLayout;
    grid->setColumnStretch(0, 0);
    grid->setColumnStretch(1, 0);
    grid->setColumnStretch(2, 1);

    grid->addWidget(new QLabel(_("Tag Group")), 0, 0, Qt::AlignLeft);
    grid->addWidget(new QLabel(" : "), 0, 1);
    grid->addWidget(&tagGroupCombo, 0, 2);
    tagGroupLabel.setVisible(false);
    grid->addWidget(&tagGroupLabel, 0, 2);

    tagGroupCombo.setEditable(true);
    tagGroupCombo.sigAboutToShowPopup().connect(
        [&](){ updateTagGroupCombo(); });
    tagGroupCombo.sigActivated().connect(
        [&](int index){ onTagGroupComboActivated(index); });

    createAdditionalInterfaces(grid);

    int row = grid->rowCount();

    grid->addWidget(new QLabel(_("Position")), row, 0, Qt::AlignLeft);
    grid->addWidget(new QLabel(" : "), row, 1);

    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect([this](){ self->onTouchup(); });
    grid->addWidget(&touchupButton, row, 2, Qt::AlignRight);

    ++row;

    auto posGrid = new QGridLayout;
    posGrid->setContentsMargins(0, 0, 0, 0);
    posGrid->setColumnMinimumWidth(0, 10);
    for(int i=0; i < 3; ++i){
        posGrid->addWidget(new QLabel(xyzCaptions[i]), 0, i * 2 + 1, Qt::AlignCenter);
        posGrid->addWidget(&xyzLabel[i], 0, i * 2 + 2, Qt::AlignCenter);
        posGrid->addWidget(new QLabel(rpyCaptions[i]), 1, i * 2 + 1, Qt::AlignCenter);
        posGrid->addWidget(&rpyLabel[i], 1, i * 2 + 2, Qt::AlignCenter);
        posGrid->setColumnStretch(i * 2 + 1, 0);
        posGrid->setColumnStretch(i * 2 + 2, 1);
    }
    grid->addLayout(posGrid, row, 0, 1, 3);
    ++row;
    
    grid->addWidget(new QLabel(_("Base")), row, 0);
    grid->addWidget(new QLabel(" : "), row, 1);
    grid->addWidget(&baseFrameLabel, row, 2);
    ++row;
    
    grid->addWidget(new QLabel(_("Tool")), row, 0);
    grid->addWidget(new QLabel(" : "), row, 1);
    grid->addWidget(&offsetFrameLabel, row, 2);
    ++row;
    
    vbox->addLayout(grid);

    jointDisplacementColumnSize = 6;
    positionLabelGridColumnSize = 1 + jointDisplacementColumnSize * 3;

    positionLabelGrid = new QGridLayout;
    positionLabelGrid->setColumnStretch(positionLabelGridColumnSize, 10);
    vbox->addLayout(positionLabelGrid);
    
    vbox->addStretch();
    
    self->setLayout(vbox);
}


MprTagTraceStatementPanel::~MprTagTraceStatementPanel()
{
    delete impl;
}


void MprTagTraceStatementPanel::setEditable(bool on)
{
    impl->isEditable = on;
    impl->tagGroupCombo.setVisible(on);
    impl->tagGroupLabel.setVisible(!on);
    impl->touchupButton.setEnabled(on);
}


void MprTagTraceStatementPanel::onActivated()
{
    impl->updateBaseInterfaces();
}


void MprTagTraceStatementPanel::onStatementUpdated()
{
    impl->updateBaseInterfaces();
}


void MprTagTraceStatementPanel::Impl::updateBaseInterfaces()
{
    updateTagGroupCombo();

    auto statement = self->currentStatement<MprTagTraceStatement>();

    auto& T = statement->tagGroupPosition();
    auto xyz = T.translation();
    if(DisplayValueFormat::instance()->isMillimeter()){
        for(int i=0; i < 3; ++i){
            xyzLabel[i].setText(QString::number(xyz[i] * 1000.0, 'f', 3));
        }
    } else {
        for(int i=0; i < 3; ++i){
            xyzLabel[i].setText(QString::number(xyz[i], 'f', 4));
        }
    }
    auto rpy = rpyFromRot(T.linear());
    for(int i=0; i < 3; ++i){
        rpyLabel[i].setText(QString::number(degree(rpy[i]), 'f', 1));
    }

    if(auto kinematicsKit = self->currentMainKinematicsKit()){
        auto baseId = statement->baseFrameId();
        auto baseFrames = kinematicsKit->baseFrames();
        MprPositionLabelSet::updateCoordinateFrameLabel(
            baseFrameLabel, baseId, baseFrames ? baseFrames->findFrame(baseId) : nullptr);
        auto offsetId = statement->offsetFrameId();
        auto offsetFrames = kinematicsKit->offsetFrames();
        MprPositionLabelSet::updateCoordinateFrameLabel(
            offsetFrameLabel, offsetId, offsetFrames ? offsetFrames->findFrame(offsetId) : nullptr);
    } else {
        baseFrameLabel.setText("---");
        offsetFrameLabel.setText("---");
    }

    int row = 0;
    int labelSetIndex = 0;
    
    if(auto subBodyPositions = statement->subBodyPositions()){
        auto bodyItemSet = self->currentBodyItemSet();
        int mainIndex = bodyItemSet->mainBodyPartIndex();
        for(auto& index : subBodyPositions->findMatchedPositionIndices(bodyItemSet)){
            if(index != mainIndex){
                auto labelSet = getOrCreatePositionLabelSet(labelSetIndex++);
                labelSet->update(
                    bodyItemSet->bodyItemPart(index),
                    subBodyPositions->position(index),
                    row,
                    positionLabelGridColumnSize,
                    jointDisplacementColumnSize,
                    true);
            }
        }
    }
    while(labelSetIndex < static_cast<int>(positionLabelSets.size())){
        positionLabelSets[labelSetIndex++]->detach();
    }
}


MprPositionLabelSet* MprTagTraceStatementPanel::Impl::getOrCreatePositionLabelSet(int index)
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


void MprTagTraceStatementPanel::Impl::updateTagGroupCombo()
{
    tagGroupCombo.blockSignals(true);
    tagGroupCombo.clear();
    int currentIndex = 0;
    bool hasCurrent = false;
    auto statement = self->currentStatement<MprTagTraceStatement>();

    if(auto worldItem = self->currentProgramItem()->findOwnerItem<WorldItem>()){
        set<string> names;
        for(auto& tagGroupItem : worldItem->descendantItems<PositionTagGroupItem>()){
            auto& name = tagGroupItem->name();
            if(names.insert(name).second){
                int index = tagGroupCombo.count();
                tagGroupCombo.addItem(name.c_str());
                if(name == statement->tagGroupName()){
                    currentIndex = index;
                    tagGroupCombo.setCurrentIndex(currentIndex);
                    hasCurrent = true;
                }
            }
        }
    }
    if(!hasCurrent){
        tagGroupCombo.insertItem(0, statement->tagGroupName().c_str());
        tagGroupCombo.setCurrentIndex(0);
    }

    tagGroupCombo.blockSignals(false);

    if(!isEditable){
        tagGroupLabel.setText(tagGroupCombo.currentText());
    }
}


void MprTagTraceStatementPanel::Impl::onTagGroupComboActivated(int comboIndex)
{
    auto statement = self->currentStatement<MprTagTraceStatement>();
    auto name = tagGroupCombo.currentText().toStdString();
    if(name != statement->tagGroupName()){
        // Clear the current tag group to force update
        statement->setTagGroup(nullptr, false, false, false);
        statement->setTagGroupName(name);
        self->currentProgramItem()->resolveStatementReferences(statement);
        self->onTouchup();
    }
}


void MprTagTraceStatementPanel::onTouchup()
{
    impl->touchupPositionAndFrames();
}


void MprTagTraceStatementPanel::Impl::touchupPositionAndFrames()
{
    auto statement = self->currentStatement<MprTagTraceStatement>();
    if(auto kinematicsKit = self->currentMainKinematicsKit()){
        statement->updateFramesWithCurrentFrames(kinematicsKit);
        if(auto tagGroupItem = PositionTagGroupItem::findItemOf(statement->tagGroup())){
            statement->updateTagGroupPositionWithGlobalCoordinate(
                kinematicsKit, tagGroupItem->originPosition());
        }
        if(isSubBodyPositionEnabled){
            statement->fetchSubBodyPositions(self->currentBodyItemSet());
        }
    }
    statement->updateTagTraceProgram();
    statement->notifyUpdate();
}
