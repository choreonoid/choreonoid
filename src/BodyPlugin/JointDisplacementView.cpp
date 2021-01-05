#include "JointDisplacementView.h"
#include "JointDisplacementWidget.h"
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
    JointDisplacementWidget jointDisplacementWidget;
    ToolButton menuButton;
    ScopedConnection bodySelectionManagerConnection;

    Impl(JointDisplacementView* self);
    void onActivated();
    void onCurrentBodyItemChanged(BodyItem* bodyItem);
};

}


void JointDisplacementView::initializeClass(ExtensionManager* ext)
{
    auto& vm = ext->viewManager();
    vm.registerClass<JointDisplacementView>(
        "JointDisplacementView", N_("Joint Displacement"), ViewManager::SINGLE_DEFAULT);
    vm.registerClassAlias("JointSliderView", "JointDisplacementView");
}


JointDisplacementView::JointDisplacementView()
{
    impl = new Impl(this);
}


JointDisplacementView::Impl::Impl(JointDisplacementView* self)
    : jointDisplacementWidget(self)
{
    self->setDefaultLayoutArea(View::CENTER);

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

    auto scrollArea = new QScrollArea;
    scrollArea->setStyleSheet("QScrollArea {background: transparent;}");
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scrollArea->setWidget(&jointDisplacementWidget);
    jointDisplacementWidget.setAutoFillBackground(false);
    vbox->addWidget(scrollArea);

    jointDisplacementWidget.sigJointWidgetFocused().connect(
        [scrollArea](QWidget* widget){ scrollArea->ensureWidgetVisible(widget); });
}


JointDisplacementView::~JointDisplacementView()
{
    delete impl;
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
    impl->jointDisplacementWidget.setBodyItem(nullptr);
}


void JointDisplacementView::onAttachedMenuRequest(MenuManager& menuManager)
{
    impl->jointDisplacementWidget.setOptionMenuTo(menuManager);
}


void JointDisplacementView::Impl::onCurrentBodyItemChanged(BodyItem* bodyItem)
{
    if(!bodyItem){
        targetLabel.setText("------");
        jointDisplacementWidget.setBodyItem(nullptr);
    } else {
        while(bodyItem){
            if(bodyItem->body()->numJoints() > 0){
                targetLabel.setText(bodyItem->displayName().c_str());
                jointDisplacementWidget.setBodyItem(bodyItem);
                break;
            }
            bodyItem = bodyItem->parentBodyItem();
        }
    }
}


bool JointDisplacementView::storeState(Archive& archive)
{
    return impl->jointDisplacementWidget.storeState(archive);
}


bool JointDisplacementView::restoreState(const Archive& archive)
{
    return impl->jointDisplacementWidget.restoreState(archive);
}
