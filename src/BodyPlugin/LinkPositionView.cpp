#include "LinkPositionView.h"
#include "LinkPositionWidget.h"
#include "BodySelectionManager.h"
#include "BodyItem.h"
#include <cnoid/ViewManager>
#include <cnoid/MenuManager>
#include <cnoid/ActionGroup>
#include <cnoid/ConnectionSet>
#include <QLabel>
#include <QStyle>
#include <QBoxLayout>
#include <QScrollArea>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class LinkPositionView::Impl
{
public:
    LinkPositionView* self;
    QLabel targetLabel;
    LinkPositionWidget* positionWidget;
    ScopedConnection activeStateConnection;

    Impl(LinkPositionView* self);
    bool setTargetBodyAndLink(BodyItem* bodyItem, Link* link);
    void onAttachedMenuRequest(MenuManager& menuManager);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void LinkPositionView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<LinkPositionView>(
        "LinkPositionView", N_("Link Position"), ViewManager::SINGLE_OPTIONAL);
}


LinkPositionView* LinkPositionView::instance()
{
    static LinkPositionView* instance_ = ViewManager::getOrCreateView<LinkPositionView>();
    return instance_;
}


LinkPositionView::LinkPositionView()
{
    impl = new Impl(this);
}


LinkPositionView::Impl::Impl(LinkPositionView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);

    auto topLayout = new QVBoxLayout;
    topLayout->setContentsMargins(0, 0, 0, 0);
    topLayout->setSpacing(0);
    self->setLayout(topLayout);

    auto style = self->style();
    int lmargin = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int rmargin = style->pixelMetric(QStyle::PM_LayoutRightMargin);
    int tmargin = style->pixelMetric(QStyle::PM_LayoutTopMargin);
    int bmargin = style->pixelMetric(QStyle::PM_LayoutBottomMargin);

    auto hbox = new QHBoxLayout;
    hbox->setContentsMargins(lmargin, tmargin / 2, rmargin, bmargin / 2);
    targetLabel.setStyleSheet("font-weight: bold");
    targetLabel.setAlignment(Qt::AlignLeft);
    hbox->addWidget(&targetLabel);
    hbox->addStretch();
    topLayout->addLayout(hbox);
    
    auto scrollArea = new QScrollArea;
    scrollArea->setStyleSheet("QScrollArea {background: transparent;}");
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setWidgetResizable(true);
    topLayout->addWidget(scrollArea);
    
    positionWidget = new LinkPositionWidget(self);
    positionWidget->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    scrollArea->setWidget(positionWidget);
    positionWidget->setAutoFillBackground(false);
}


LinkPositionView::~LinkPositionView()
{
    delete impl;
}


void LinkPositionView::onActivated()
{
    auto bsm = BodySelectionManager::instance();
    
    impl->activeStateConnection =
        bsm->sigCurrentSpecified().connect(
            [&](BodyItem* bodyItem, Link* link){
                impl->setTargetBodyAndLink(bodyItem, link);
            });
}


bool LinkPositionView::Impl::setTargetBodyAndLink(BodyItem* bodyItem, Link* link)
{
    positionWidget->setTargetBodyAndLink(bodyItem, link);
    auto targetBodyItem = positionWidget->targetBodyItem();
    if(targetBodyItem){
        targetLabel.setText(
            format("{0} / {1}",
                   targetBodyItem->displayName(),
                   positionWidget->targetLink()->name()).c_str());
    } else {
        targetLabel.setText("------");
    }
    return (targetBodyItem != nullptr);
}


void LinkPositionView::onDeactivated()
{
    impl->activeStateConnection.disconnect();
}


void LinkPositionView::onAttachedMenuRequest(MenuManager& menu)
{
    menu.setPath("/").setPath(_("Target link type"));
    auto checkGroup = new ActionGroup(menu.topMenu());
    menu.addRadioItem(checkGroup, _("Any links"));
    menu.addRadioItem(checkGroup, _("IK priority link and root link"));
    menu.addRadioItem(checkGroup, _("IK priority link"));
    checkGroup->actions()[impl->positionWidget->targetLinkType()]->setChecked(true);
    checkGroup->sigTriggered().connect(
        [=](QAction* check){
            impl->positionWidget->setTargetLinkType(checkGroup->actions().indexOf(check)); });
                                           
    menu.setPath("/");
    menu.addSeparator();
    
    impl->positionWidget->setOptionMenuTo(menu);
}
    

bool LinkPositionView::storeState(Archive& archive)
{
    impl->positionWidget->storeState(archive);
    
    switch(impl->positionWidget->targetLinkType()){
    case LinkPositionWidget::AnyLink:
        archive.write("target_link_type", "any_link");
        break;
    case LinkPositionWidget::RootOrIkLink:
        archive.write("target_link_type", "root_or_ik_link");
        break;
    case LinkPositionWidget::IkLink:
        archive.write("target_link_type", "ik_link");
        break;
    }
    return true;
}


bool LinkPositionView::restoreState(const Archive& archive)
{
    impl->positionWidget->restoreState(archive);
    
    string type;
    if(archive.read("target_link_type", type)){
        if(type == "any_link"){
            impl->positionWidget->setTargetLinkType(LinkPositionWidget::AnyLink);
        } else if(type == "root_or_ik_link"){
            impl->positionWidget->setTargetLinkType(LinkPositionWidget::RootOrIkLink);
        } else if(type == "ik_link"){
            impl->positionWidget->setTargetLinkType(LinkPositionWidget::IkLink);
        }
    }
    return true;
}
