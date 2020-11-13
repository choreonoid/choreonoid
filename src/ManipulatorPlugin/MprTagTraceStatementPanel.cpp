#include "MprTagTraceStatementPanel.h"
#include "MprPositionStatementPanel.h"
#include "MprTagTraceStatement.h"
#include "MprProgramItemBase.h"
#include <cnoid/RootItem>
#include <cnoid/PositionTagGroupItem>
#include <cnoid/ItemList>
#include <cnoid/LinkKinematicsKit>
#include <cnoid/CoordinateFrameList>
#include <cnoid/MessageView>
#include <cnoid/EigenUtil>
#include <cnoid/Buttons>
#include <cnoid/ComboBox>
#include <QLabel>
#include <QBoxLayout>
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

class MprTagTraceStatementPanel::Impl
{
public:
    MprTagTraceStatementPanel* self;
    bool isEditingEnabled;
    QGridLayout* grid;
    ComboBox tagGroupCombo;
    QLabel tagGroupLabel;
    vector<weak_ref_ptr<PositionTagGroupItem>> tagGroupCandidates;
    PushButton touchupButton;
    QLabel xyzLabel[3];
    QLabel rpyLabel[3];
    QLabel baseFrameLabel;
    QLabel offsetFrameLabel;

    Impl(MprTagTraceStatementPanel* self);
    void createBaseInterfaces(
        const std::function<void(QGridLayout* grid)>& createAdditionalInterfaces);
    void updateBaseInterfaces();
    void updateTagGroupCandidates();
    void onTagGroupComboActivated(int comboIndex);
    PositionTagGroupItem* getTagGroupItemOf(PositionTagGroup* tagGroup);
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
    isEditingEnabled = true;
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

    tagGroupCombo.sigAboutToShowPopup().connect(
        [&](){ updateTagGroupCandidates(); });
    tagGroupCombo.sigActivated().connect(
        [&](int index){ onTagGroupComboActivated(index); });

    createAdditionalInterfaces(grid);

    int row = grid->rowCount();

    grid->addWidget(new QLabel(_("Position ")), row, 0, Qt::AlignLeft);
    grid->addWidget(new QLabel(" : "), row, 1);

    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect(
        [&](){ touchupPositionAndFrames(); });
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
    vbox->addStretch();
    
    self->setLayout(vbox);
}


MprTagTraceStatementPanel::~MprTagTraceStatementPanel()
{
    delete impl;
}


void MprTagTraceStatementPanel::setEditingEnabled(bool on)
{
    impl->isEditingEnabled = on;
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
    updateTagGroupCandidates();

    auto programItem = self->currentProgramItem();
    auto kinematicsKit = programItem->kinematicsKit();
    auto statement = self->currentStatement<MprTagTraceStatement>();

    auto& T = statement->tagGroupPosition();
    auto xyz = T.translation();
    auto rpy = rpyFromRot(T.linear());
    for(int i=0; i < 3; ++i){
        xyzLabel[i].setText(QString::number(xyz[i], 'f', 4));
        rpyLabel[i].setText(QString::number(degree(rpy[i]), 'f', 1));
    }

    auto baseId = statement->baseFrameId();
    MprPositionStatementPanel::updateCoordinateFrameLabel(
        baseFrameLabel, baseId, kinematicsKit->baseFrames()->findFrame(baseId), kinematicsKit->baseFrames());
    auto offsetId = statement->offsetFrameId();
    MprPositionStatementPanel::updateCoordinateFrameLabel(
        offsetFrameLabel, offsetId, kinematicsKit->offsetFrames()->findFrame(offsetId), kinematicsKit->offsetFrames());
}


void MprTagTraceStatementPanel::Impl::updateTagGroupCandidates()
{
    tagGroupCombo.blockSignals(true);
    tagGroupCandidates.clear();
    tagGroupCombo.clear();
    int currentIndex = 0;
    bool hasCurrent = false;
    auto currentTagGroup = self->currentStatement<MprTagTraceStatement>()->tagGroup();
    
    for(auto& tagGroupItem : RootItem::instance()->descendantItems<PositionTagGroupItem>()){
        int index = tagGroupCandidates.size();
        tagGroupCandidates.push_back(tagGroupItem);
        tagGroupCombo.addItem(tagGroupItem->name().c_str(), index);
        if(tagGroupItem->tagGroup() == currentTagGroup){
            currentIndex = index;
            hasCurrent = true;
        }
    }
    if(hasCurrent){
        tagGroupCombo.setCurrentIndex(currentIndex);

    } else if(!tagGroupCandidates.empty()){
        tagGroupCombo.insertItem(0, "", -1);
        tagGroupCombo.setCurrentIndex(0);
    }

    tagGroupCombo.blockSignals(false);

    if(!isEditingEnabled){
        tagGroupLabel.setText(tagGroupCombo.currentText());
    }
}


void MprTagTraceStatementPanel::Impl::onTagGroupComboActivated(int comboIndex)
{
    bool isValid = true;
    bool updated = false;
    
    PositionTagGroupItem* tagGroupItem = nullptr;
    if(comboIndex >= 0){
        int candidateIndex = tagGroupCombo.itemData(comboIndex).toInt();
        if(candidateIndex >= 0){
            tagGroupItem = tagGroupCandidates[candidateIndex].lock();
        }
    }
    auto statement = self->currentStatement<MprTagTraceStatement>();
    if(!tagGroupItem){
        statement->setTagGroup(nullptr);
        statement->setTagGroupPosition(Position::Identity());
        statement->updateTagTraceProgram();
        updated = true;
    } else {
        if(statement->tagGroup() != tagGroupItem->tagGroup()){
            statement->setTagGroup(tagGroupItem->tagGroup());
            auto kinematicsKit = self->currentProgramItem()->kinematicsKit();
            statement->updateTagGroupPositionWithGlobalParentCoordinateSystem(
                kinematicsKit, tagGroupItem->parentCoordinateSystem());
            isValid = statement->updateTagTraceProgram();
            updated = true;
        }
    }
    if(!isValid){
        showWarningDialog(
            format(_("The program to trace {0} cannot be generated."),
                   tagGroupItem->displayName()));
    }
    if(updated){
        statement->notifyUpdate();
    }
}


PositionTagGroupItem* MprTagTraceStatementPanel::Impl::getTagGroupItemOf(PositionTagGroup* tagGroup)
{
    //updateTagGroupCandidates();
    for(auto& candidate : tagGroupCandidates){
        if(auto item = candidate.lock()){
            if(item->tagGroup() == tagGroup){
                return item;
            }
        }
    }
    return nullptr;
}


void MprTagTraceStatementPanel::Impl::touchupPositionAndFrames()
{
    auto statement = self->currentStatement<MprTagTraceStatement>();
    auto kinematicsKit = self->currentProgramItem()->kinematicsKit();
    statement->updateFramesWithCurrentFrames(kinematicsKit);
    if(auto tagGroupItem = getTagGroupItemOf(statement->tagGroup())){
        statement->updateTagGroupPositionWithGlobalParentCoordinateSystem(
            kinematicsKit, tagGroupItem->parentCoordinateSystem());
    }
    statement->notifyUpdate();
}
