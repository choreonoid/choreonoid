/**
   @author Shin'ichiro Nakaoka
*/

#include "LinkSelectionView.h"
#include "LinkTreeWidget.h"
#include "BodySelectionManager.h"
#include <cnoid/ViewManager>
#include <QBoxLayout>
#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {
LinkSelectionView* instance = nullptr;
}

namespace cnoid {

class LinkSelectionView::Impl
{
public:
    LinkTreeWidget linkTreeWidget;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionManagerConnection;
    ScopedConnection linkTreeWidgetConnection;

    Impl(LinkSelectionView* self);
    void onBodySelectionManagerCurrentChanged(BodyItem* bodyItem, Link* link);
    void onLinkTreeWidgetSelectionChanged();
};

}


void LinkSelectionView::initializeClass(ExtensionManager* ext)
{
    ::instance =
        ext->viewManager().registerClass<LinkSelectionView>(
            "LinkSelectionView", N_("Links"), ViewManager::SINGLE_DEFAULT);
}


LinkSelectionView* LinkSelectionView::instance()
{
    return ::instance;
}


LinkSelectionView::LinkSelectionView()
{
    impl = new Impl(this);
}


LinkSelectionView::Impl::Impl(LinkSelectionView* self)
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

    bodySelectionManager = BodySelectionManager::instance();

    bodySelectionManagerConnection.reset(
        bodySelectionManager->sigCurrentChanged().connect(
            [&](BodyItem* bodyItem, Link* link){
                onBodySelectionManagerCurrentChanged(bodyItem, link); }));

    linkTreeWidgetConnection.reset(
        linkTreeWidget.sigSelectionChanged().connect(
            [&](){ onLinkTreeWidgetSelectionChanged(); }));
}


LinkSelectionView::~LinkSelectionView()
{
    delete impl;
}


void LinkSelectionView::Impl::onBodySelectionManagerCurrentChanged(BodyItem* bodyItem, Link* link)
{
    linkTreeWidget.setBodyItem(bodyItem);
    if(bodyItem){
        if(link){
            linkTreeWidget.makeSingleSelection(bodyItem, link->index());
        } else {
            //linkTreeWidget.setSelection(bodySelectionManager->linkSelection(bodyItem));
        }
    }
}


BodyItem* LinkSelectionView::currentBodyItem()
{
    return impl->linkTreeWidget.bodyItem();
}


SignalProxy<void()> LinkSelectionView::sigSelectionChanged()
{
    return impl->linkTreeWidget.sigSelectionChanged();
}


void LinkSelectionView::Impl::onLinkTreeWidgetSelectionChanged()
{
    bodySelectionManagerConnection.block();

    bodySelectionManager->setLinkSelection(
        linkTreeWidget.bodyItem(), linkTreeWidget.linkSelection());
    
    bodySelectionManagerConnection.unblock();
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
