#include "MprPositionListView.h"
#include "MprProgramItemBase.h"
#include "MprPositionListWidget.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/TargetItemPicker>
#include <cnoid/BodyItem>
#include <cnoid/Archive>
#include <cnoid/Buttons>
#include <cnoid/Format>
#include <QBoxLayout>
#include <QLabel>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

MprPositionListWidget::BodySyncMode defaultBodySyncMode = MprPositionListWidget::DirectBodySync;

}

namespace cnoid {

class MprPositionListView::Impl
{
public:
    MprPositionListView* self;
    MprPositionListWidget* positionListWidget;
    TargetItemPicker<MprProgramItemBase> targetItemPicker;
    MprProgramItemBasePtr programItem;
    QLabel targetLabel;
    PushButton addButton;
    PushButton touchupButton;

    Impl(MprPositionListView* self);
    void setProgramItem(MprProgramItemBase* item);
};

}


void MprPositionListView::setDefaultBodySyncMode(BodySyncMode mode)
{
    defaultBodySyncMode = static_cast<MprPositionListWidget::BodySyncMode>(mode);
}


void MprPositionListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<MprPositionListView>(
        N_("MprPositionListView"), N_("Waypoints"));
}


MprPositionListView::MprPositionListView()
{
    impl = new Impl(this);
}


MprPositionListView::Impl::Impl(MprPositionListView* self)
    : self(self),
      targetItemPicker(self)
{
    self->setDefaultLayoutArea(BottomCenterArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    auto sty = self->style();
    int hs = sty->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    hbox->addSpacing(hs);
    targetLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&targetLabel, 0, Qt::AlignVCenter);
    hbox->addStretch();
    addButton.setText(_("Add"));
    addButton.sigClicked().connect([this](){
        positionListWidget->addPositionIntoCurrentIndex(false);
    });
    hbox->addWidget(&addButton);
    touchupButton.setText(_("Touch-up"));
    touchupButton.sigClicked().connect([this](){
        positionListWidget->touchupCurrentPosition();
    });
    hbox->addWidget(&touchupButton);
    vbox->addLayout(hbox);

    auto hframe = new QFrame;
    hframe->setFrameStyle(static_cast<int>(QFrame::HLine) | static_cast<int>(QFrame::Sunken));
    vbox->addWidget(hframe);
    
    positionListWidget = new MprPositionListWidget(self);
    positionListWidget->setStandardUserOperationEnabled(true);
    positionListWidget->setBodySyncMode(defaultBodySyncMode);
    positionListWidget->setFrameShape(QFrame::NoFrame);
    vbox->addWidget(positionListWidget);
    
    self->setLayout(vbox);

    targetItemPicker.sigTargetItemChanged().connect(
        [this](MprProgramItemBase* item){
            setProgramItem(item);
        });
}


MprPositionListView::~MprPositionListView()
{
    delete impl;
}


void MprPositionListView::onAttachedMenuRequest(MenuManager& menuManager)
{
    auto twoStageCheck = menuManager.addCheckItem(_("Two-stage sync"));
    auto bodySyncMode = impl->positionListWidget->bodySyncMode();
    twoStageCheck->setChecked(bodySyncMode == MprPositionListWidget::TwoStageBodySync);
    twoStageCheck->sigToggled().connect(
        [this](bool on){
            impl->positionListWidget->setBodySyncMode(
                on ? MprPositionListWidget::TwoStageBodySync : MprPositionListWidget::DirectBodySync);
        });
}


void MprPositionListView::Impl::setProgramItem(MprProgramItemBase* item)
{
    programItem = item;

    if(item){
        string caption;
        if(auto bodyItem = item->targetBodyItem()){
            targetLabel.setText(
                formatC("{0} - {1}",  bodyItem->displayName(), item->displayName()).c_str());
        } else {
            targetLabel.setText(item->displayName().c_str());
        }
        positionListWidget->setBodyItemSet(item->targetBodyItemSet());
        positionListWidget->setPositionList(item->program()->positionList());
    } else {
        targetLabel.setText("---");
        positionListWidget->setBodyItemSet(nullptr);
        positionListWidget->setPositionList(nullptr);
    }
    addButton.setEnabled(programItem != nullptr);
}

      
void MprPositionListView::setBodySyncMode(BodySyncMode mode)
{
    impl->positionListWidget->setBodySyncMode(
        static_cast<MprPositionListWidget::BodySyncMode>(mode));
}


bool MprPositionListView::storeState(Archive& archive)
{
    impl->targetItemPicker.storeTargetItem(archive, "current_item");
    auto mode = static_cast<BodySyncMode>(impl->positionListWidget->bodySyncMode());
    if(mode == DirectBodySync){
        archive.write("body_sync_mode", "direct");
    } else if(mode == TwoStageBodySync){
        archive.write("body_sync_mode", "two-stage");
    }
    return true;
}


bool MprPositionListView::restoreState(const Archive& archive)
{
    impl->targetItemPicker.restoreTargetItemLater(archive, "current_item");
    string mode;
    if(archive.read("body_sync_mode", mode)){
        if(mode == "direct"){
            impl->positionListWidget->setBodySyncMode(MprPositionListWidget::DirectBodySync);
        } else if(mode == "two-stage"){
            impl->positionListWidget->setBodySyncMode(MprPositionListWidget::TwoStageBodySync);
        }
    }
    return true;
}
