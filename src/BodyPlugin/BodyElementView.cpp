#include "BodyElementView.h"
#include "BodyElementTreeWidget.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/ComboBox>
#include <QBoxLayout>
//#include <QHeaderView>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodyElementView::Impl
{
public:
    BodyElementTreeWidget treeWidget;
    ScopedConnection treeWidgetConnection;
    ComboBox listingModeCombo;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionManagerConnection;

    Impl(BodyElementView* self);
    void onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link);
    void onTreeWidgetLinkSelectionChanged();
};

}


void BodyElementView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<BodyElementView>(
        "BodyElementView", N_("Links / Devices"), ViewManager::SINGLE_OPTIONAL);
}


BodyElementView* BodyElementView::instance()
{
    static BodyElementView* instance_ = ViewManager::getOrCreateView<BodyElementView>();
    return instance_;
}


BodyElementView::BodyElementView()
{
    impl = new Impl(this);
}


BodyElementView::Impl::Impl(BodyElementView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout();
    vbox->setSpacing(0);

    listingModeCombo.addItem(_("Link / Device Tree"), BodyElementTreeWidget::Tree);
    listingModeCombo.addItem(_("Link List"), BodyElementTreeWidget::LinkList);
    listingModeCombo.addItem(_("Joint List"), BodyElementTreeWidget::JointList);
    listingModeCombo.addItem(_("Device List"), BodyElementTreeWidget::DeviceList);
    listingModeCombo.addItem(_("Body Part Hierarchy"), BodyElementTreeWidget::PartTree);
    listingModeCombo.sigCurrentIndexChanged().connect(
        [&](int){ treeWidget.setListingMode(listingModeCombo.currentData().toInt()); });

    vbox->addWidget(&listingModeCombo);

    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setListingMode(BodyElementTreeWidget::LinkList);
    treeWidget.enableCache(true);
    vbox->addWidget(&treeWidget);
    
    self->setLayout(vbox);

    bodySelectionManager = BodySelectionManager::instance();

    bodySelectionManagerConnection =
        bodySelectionManager->sigCurrentChanged().connect(
            [&](BodyItem* bodyItem, Link* link){
                onCurrentBodySelectionChanged(bodyItem, link); });

    treeWidgetConnection =
        treeWidget.sigLinkSelectionChanged().connect(
            [&](){ onTreeWidgetLinkSelectionChanged(); });

    onCurrentBodySelectionChanged(
        bodySelectionManager->currentBodyItem(), bodySelectionManager->currentLink());
}


BodyElementView::~BodyElementView()
{
    delete impl;
}


BodyItem* BodyElementView::currentBodyItem()
{
    return impl->treeWidget.bodyItem();
}


SignalProxy<void(int linkIndex)> BodyElementView::sigCurrentLinkChanged()
{
    return impl->treeWidget.sigCurrentLinkChanged();
}


int BodyElementView::currentLinkIndex() const
{
    return impl->treeWidget.currentLinkIndex();
}


void BodyElementView::setCurrentLink(int index)
{
    impl->treeWidget.setCurrentLink(index);
}


SignalProxy<void()> BodyElementView::sigLinkSelectionChanged()
{
    return impl->treeWidget.sigLinkSelectionChanged();
}


const std::vector<int>& BodyElementView::selectedLinkIndices() const
{
    return impl->treeWidget.selectedLinkIndices();
}


const std::vector<bool>& BodyElementView::linkSelection() const
{
    return impl->treeWidget.linkSelection();
}


void BodyElementView::Impl::onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link)
{
    treeWidget.setBodyItem(bodyItem);
    if(bodyItem && link){
        treeWidget.setCurrentLink(link->index());
    }
}


void BodyElementView::Impl::onTreeWidgetLinkSelectionChanged()
{
    bodySelectionManagerConnection.block();
    bodySelectionManager->setLinkSelection(
        treeWidget.bodyItem(), treeWidget.linkSelection());
    bodySelectionManagerConnection.unblock();
}


bool BodyElementView::storeState(Archive& archive)
{
    const char* mode = nullptr;
    switch(impl->treeWidget.listingMode()){
    case BodyElementTreeWidget::Tree:       mode = "tree";   break;
    case BodyElementTreeWidget::LinkList:   mode = "link";   break;
    case BodyElementTreeWidget::JointList:  mode = "joint";  break;
    case BodyElementTreeWidget::DeviceList: mode = "device"; break;
    case BodyElementTreeWidget::PartTree:   mode = "part";   break;
    default: break;
    }
    if(mode){
        archive.write("listing_mode", mode);
    }
    if(auto bodyItem = impl->treeWidget.bodyItem()){
        archive.writeItemId("current_body_item", bodyItem);
    }
    return impl->treeWidget.storeState(archive);
}


bool BodyElementView::restoreState(const Archive& archive)
{
    int mode = BodyElementTreeWidget::Tree;
    string modeString;
    if(archive.read("listing_mode", modeString)){
        if(modeString == "link"){
            mode = BodyElementTreeWidget::LinkList;
        } else if(modeString == "joint"){
            mode = BodyElementTreeWidget::JointList;
        } else if(modeString == "device"){
            mode = BodyElementTreeWidget::DeviceList;
        } else if(modeString == "part"){
            mode = BodyElementTreeWidget::PartTree;
        }
    }
    int modeIndex = impl->listingModeCombo.findData(mode);
    if(modeIndex >= 0){
        impl->listingModeCombo.blockSignals(true);
        impl->listingModeCombo.setCurrentIndex(modeIndex);
        impl->listingModeCombo.blockSignals(false);
        impl->treeWidget.setListingMode(mode);
    }

    if(impl->treeWidget.restoreState(archive)){
        archive.addPostProcess(
            [this, &archive](){
                if(auto item = archive.findItem<BodyItem>("current_body_item")){
                    impl->treeWidget.setBodyItem(item);
                }
            });
        return true;
    }

    return false;
}
