#include "JointDisplacementView.h"
#include "JointDisplacementWidgetSet.h"
#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/Archive>
#include <cnoid/ViewManager>
#include <cnoid/Buttons>
#include <QLabel>
#include <QScrollArea>
#include <QBoxLayout>
#include <QStyle>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class JointDisplacementView::Impl
{
public:
    QLabel targetLabel;
    JointDisplacementWidgetSet* displacementWidgetSet;
    ToolButton menuButton;
    ScopedConnection bodySelectionManagerConnection;
    ScopedConnection modelUpdateConnection;

    Impl(JointDisplacementView* self);
    ~Impl();
    void onActivated();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
};

}


void JointDisplacementView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<JointDisplacementView>(
        N_("JointDisplacementView"), N_("Joint Displacement"));
    ViewManager::setClassAlias("JointSliderView", "JointDisplacementView");
}


JointDisplacementView::JointDisplacementView()
{
    impl = new Impl(this);
}


JointDisplacementView::Impl::Impl(JointDisplacementView* self)
{
    self->setDefaultLayoutArea(BottomRightArea);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(lmargin, tmargin / 2, rmargin, bmargin / 2);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&targetLabel);
    hbox->addStretch();
    vbox->addLayout(hbox);

    auto baseWidget = new QWidget;
    displacementWidgetSet = new JointDisplacementWidgetSet(baseWidget);
    displacementWidgetSet->setSelectedJointsOnlyModeEnabled(true);

    auto scrollArea = new QScrollArea;
    scrollArea->setStyleSheet("QScrollArea {background: transparent;}");
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidget(baseWidget);
    baseWidget->setAutoFillBackground(false);
    vbox->addWidget(scrollArea);

    displacementWidgetSet->sigJointWidgetFocused().connect(
        [scrollArea](QWidget* widget){ scrollArea->ensureWidgetVisible(widget); });
}


JointDisplacementView::~JointDisplacementView()
{
    delete impl;
}


JointDisplacementView::Impl::~Impl()
{
    delete displacementWidgetSet;
}


void JointDisplacementView::onActivated()
{
    impl->onActivated();
}


void JointDisplacementView::Impl::onActivated()
{
    auto bsm = BodySelectionManager::instance();

    bodySelectionManagerConnection = 
        bsm->sigCurrentBodyItemChanged().connect(
            [&](BodyItem* bodyItem){ onCurrentBodyItemChanged(bodyItem); });

    onCurrentBodyItemChanged(bsm->currentBodyItem());
}


void JointDisplacementView::onDeactivated()
{
    impl->bodySelectionManagerConnection.disconnect();
    impl->displacementWidgetSet->setBodyItem(nullptr);
    impl->modelUpdateConnection.disconnect();
}


void JointDisplacementView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->displacementWidgetSet->setOptionMenuTo(menuManager);
}


void JointDisplacementView::Impl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    modelUpdateConnection.disconnect();
    
    if(!bodyItem){
        targetLabel.setText("------");
        displacementWidgetSet->setBodyItem(nullptr);
    } else {
        bool updated = false;
        auto candidate = bodyItem;
        while(candidate){
            if(candidate->body()->numAllJoints() > 0){
                targetLabel.setText(candidate->displayName().c_str());
                displacementWidgetSet->setBodyItem(candidate);
                updated = true;
                break;
            }
            candidate = candidate->parentBodyItem();
        }
        if(!updated){
            modelUpdateConnection =
                bodyItem->sigModelUpdated().connect(
                    [this, bodyItem](int flags){
                        if(flags & (BodyItem::LinkSetUpdate | BodyItem::LinkSpecUpdate)){
                            onCurrentBodyItemChanged(bodyItem);
                        }
                    });
        }
    }
}


bool JointDisplacementView::storeState(Archive& archive)
{
    return impl->displacementWidgetSet->storeState(&archive);
}


bool JointDisplacementView::restoreState(const Archive& archive)
{
    return impl->displacementWidgetSet->restoreState(&archive);
}
