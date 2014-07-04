/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkSelectionView.h"
#include "LinkTreeWidget.h"
#include "BodyBar.h"
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <QHeaderView>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
LinkSelectionView* mainLinkSelectionView = 0;
}

namespace cnoid {

class LinkSelectionViewImpl
{
public:
    LinkSelectionViewImpl(LinkSelectionView* self);
    ~LinkSelectionViewImpl();
    LinkTreeWidget linkTreeWidget;
    Connection currentBodyItemChangeConnection;
};

}


void LinkSelectionView::initializeClass(ExtensionManager* ext)
{
    mainLinkSelectionView =
        ext->viewManager().registerClass<LinkSelectionView>(
            "LinkSelectionView", N_("Links"), ViewManager::SINGLE_DEFAULT);
}


LinkSelectionView* LinkSelectionView::mainInstance()
{
    return mainLinkSelectionView;
}


LinkSelectionView::LinkSelectionView()
{
    impl = new LinkSelectionViewImpl(this);
}


LinkSelectionViewImpl::LinkSelectionViewImpl(LinkSelectionView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    linkTreeWidget.setFrameShape(QFrame::NoFrame);
    linkTreeWidget.enableCache(true);
    linkTreeWidget.enableArchiveOfCurrentBodyItem(true);
    linkTreeWidget.setListingMode(LinkTreeWidget::LINK_LIST);

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setSpacing(0);
    vbox->addWidget(linkTreeWidget.listingModeCombo());
    vbox->addWidget(&linkTreeWidget);
    self->setLayout(vbox);

    currentBodyItemChangeConnection =
        BodyBar::instance()->sigCurrentBodyItemChanged().connect(
            boost::bind(&LinkTreeWidget::setBodyItem, &linkTreeWidget, _1));
}


LinkSelectionView::~LinkSelectionView()
{
    delete impl;
}


LinkSelectionViewImpl::~LinkSelectionViewImpl()
{
    currentBodyItemChangeConnection.disconnect();
}


SignalProxy<void()> LinkSelectionView::sigSelectionChanged(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.sigSelectionChanged(bodyItem);
}


const std::vector<int>& LinkSelectionView::getSelectedLinkIndices(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.getSelectedLinkIndices(bodyItem);
}


const boost::dynamic_bitset<>& LinkSelectionView::getLinkSelection(BodyItemPtr bodyItem)
{
    return impl->linkTreeWidget.getLinkSelection(bodyItem);
}


bool LinkSelectionView::makeSingleSelection(BodyItemPtr bodyItem, int linkIndex)
{
    return impl->linkTreeWidget.makeSingleSelection(bodyItem, linkIndex);
}


bool LinkSelectionView::storeState(Archive& archive)
{
    return impl->linkTreeWidget.storeState(archive);
}


bool LinkSelectionView::restoreState(const Archive& archive)
{
    return impl->linkTreeWidget.restoreState(archive);
}
