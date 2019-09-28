/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkSelectionView.h"
#include "LinkTreeWidget.h"
#include "BodyBar.h"
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <QHeaderView>
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


LinkSelectionView* LinkSelectionView::instance()
{
    return mainLinkSelectionView;
}


LinkSelectionView* LinkSelectionView::mainInstance()
{
    return instance();
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
            [&](BodyItem* bodyItem){ linkTreeWidget.setBodyItem(bodyItem); });
}


LinkSelectionView::~LinkSelectionView()
{
    delete impl;
}


LinkSelectionViewImpl::~LinkSelectionViewImpl()
{
    currentBodyItemChangeConnection.disconnect();
}


BodyItem* LinkSelectionView::currentBodyItem()
{
    return impl->linkTreeWidget.bodyItem();
}


SignalProxy<void()> LinkSelectionView::sigSelectionChanged()
{
    return impl->linkTreeWidget.sigSelectionChanged();
}


int LinkSelectionView::selectedLinkIndex() const
{
    return impl->linkTreeWidget.selectedLinkIndex();
}


const std::vector<int>& LinkSelectionView::selectedLinkIndices() const
{
    return impl->linkTreeWidget.selectedLinkIndices();
}


const std::vector<bool>& LinkSelectionView::linkSelection() const
{
    return impl->linkTreeWidget.linkSelection();
}


SignalProxy<void()> LinkSelectionView::sigSelectionChanged(BodyItem* bodyItem)
{
    return impl->linkTreeWidget.sigSelectionChanged(bodyItem);
}


int LinkSelectionView::selectedLinkIndex(BodyItem* bodyItem) const
{
    return impl->linkTreeWidget.selectedLinkIndex(bodyItem);
}


const std::vector<int>& LinkSelectionView::selectedLinkIndices(BodyItem* bodyItem) const
{
    return impl->linkTreeWidget.selectedLinkIndices(bodyItem);
}


const std::vector<bool>& LinkSelectionView::linkSelection(BodyItem* bodyItem) const
{
    return impl->linkTreeWidget.linkSelection(bodyItem);
}


bool LinkSelectionView::makeSingleSelection(BodyItem* bodyItem, int linkIndex)
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
