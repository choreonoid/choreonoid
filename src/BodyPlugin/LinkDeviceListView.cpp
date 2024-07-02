#include "LinkDeviceListView.h"
#include "LinkDeviceTreeWidget.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/Archive>
#include <cnoid/ComboBox>
#include <QBoxLayout>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class LinkDeviceListView::Impl
{
public:
    LinkDeviceTreeWidget treeWidget;
    bool needToInitializeTreeWidgetElementTypeAndListingMode;
    ScopedConnection treeWidgetConnection;
    enum ElementType { ALL, LINK, JOINT, DEVICE };
    ComboBox elementTypeCombo;
    ComboBox listingModeCombo;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionManagerConnection;
    ScopedConnection bodyItemConnection;

    Impl(LinkDeviceListView* self);
    void setElementType(int type, bool doUpdate);
    void setListingMode(int mode, bool doUpdate);
    void setCurrentBodyItem(BodyItem* bodyItem, Link* link);
    void onTreeWidgetLinkSelectionChanged();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void LinkDeviceListView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager()
        .registerClass<LinkDeviceListView>(N_("LinkDeviceListView"), N_("Links / Devices"));
    ViewManager::setClassAlias("LinkSelectionView", "Body::LinkDeviceListView");
}


LinkDeviceListView::LinkDeviceListView()
{
    impl = new Impl(this);
}


LinkDeviceListView::Impl::Impl(LinkDeviceListView* self)
{
    self->setDefaultLayoutArea(BottomLeftArea);
    self->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);

    auto hbox = new QHBoxLayout;
    elementTypeCombo.addItem(_("All"));
    elementTypeCombo.addItem(_("Links"));
    elementTypeCombo.addItem(_("Joints"));
    elementTypeCombo.addItem(_("Devices"));
    hbox->addWidget(&elementTypeCombo);

    listingModeCombo.addItem(_("List"));
    listingModeCombo.addItem(_("Tree"));
    listingModeCombo.addItem(_("Grouped Tree"));
    hbox->addWidget(&listingModeCombo);
    vbox->addLayout(hbox);

    auto hframe = new QFrame;
    hframe->setFrameStyle(static_cast<int>(QFrame::HLine) | static_cast<int>(QFrame::Sunken));
    vbox->addWidget(hframe);

    treeWidget.setCacheEnabled(true);
    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setVerticalGridLineShown(true);
    vbox->addWidget(&treeWidget);
    
    self->setLayout(vbox);

    needToInitializeTreeWidgetElementTypeAndListingMode = true;

    elementTypeCombo.sigCurrentIndexChanged().connect(
        [&](int index){ setElementType(index, true); });

    listingModeCombo.sigCurrentIndexChanged().connect(
        [&](int index){ setListingMode(index, true); });

    treeWidgetConnection =
        treeWidget.sigLinkSelectionChanged().connect(
            [&](){ onTreeWidgetLinkSelectionChanged(); });

    bodySelectionManager = BodySelectionManager::instance();
}


LinkDeviceListView::~LinkDeviceListView()
{
    delete impl;
}


void LinkDeviceListView::onActivated()
{
    auto bsm = impl->bodySelectionManager;
    
    impl->bodySelectionManagerConnection =
        bsm->sigCurrentChanged().connect(
            [&](BodyItem* bodyItem, Link* link){
                impl->setCurrentBodyItem(bodyItem, link); });
    
    impl->setCurrentBodyItem(bsm->currentBodyItem(), bsm->currentLink());
}


void LinkDeviceListView::onDeactivated()
{
    impl->bodySelectionManagerConnection.disconnect();
}


void LinkDeviceListView::Impl::setElementType(int type, bool doUpdate)
{
    if(type == DEVICE && listingModeCombo.currentIndex() != LinkDeviceTreeWidget::List){
        listingModeCombo.blockSignals(true);
        listingModeCombo.setCurrentIndex(LinkDeviceTreeWidget::List);
        setListingMode(LinkDeviceTreeWidget::List, false);
        listingModeCombo.blockSignals(false);
    }
    
    treeWidget.setLinkItemVisible(type == ALL || type == LINK);
    treeWidget.setJointItemVisible(type == JOINT);
    treeWidget.setDeviceItemVisible(type == ALL || type == DEVICE);

    if(type == LINK){
        treeWidget.setNumberColumnMode(LinkDeviceTreeWidget::Index);
    } else if(type != ALL){
        treeWidget.setNumberColumnMode(LinkDeviceTreeWidget::Identifier);
    }

    if(doUpdate){
        treeWidget.updateTreeItems();
    }
}


void LinkDeviceListView::Impl::setListingMode(int mode, bool doUpdate)
{
    treeWidget.setListingMode(mode);

    if(mode != LinkDeviceTreeWidget::List){
        if(elementTypeCombo.currentIndex() == DEVICE){
            elementTypeCombo.blockSignals(true);
            elementTypeCombo.setCurrentIndex(ALL);
            treeWidget.setLinkItemVisible(true);
            treeWidget.setDeviceItemVisible(true);
            setElementType(ALL, false);
            elementTypeCombo.blockSignals(false);
        }
    }

    if(doUpdate){
        treeWidget.updateTreeItems();
    }
}    


void LinkDeviceListView::Impl::setCurrentBodyItem(BodyItem* bodyItem, Link* link)
{
    if(needToInitializeTreeWidgetElementTypeAndListingMode){
        setElementType(elementTypeCombo.currentIndex(), false);
        setListingMode(listingModeCombo.currentIndex(), false);
        needToInitializeTreeWidgetElementTypeAndListingMode = false;
    }
    if(bodyItem && link){
        treeWidget.setLinkSelection(bodyItem, bodySelectionManager->linkSelection(bodyItem));
    }
    treeWidget.setBodyItem(bodyItem);

    bodyItemConnection.disconnect();

    if(bodyItem){
        bodyItemConnection =
            bodyItem->sigModelUpdated().connect(
                [this](int flags){
                    if(((flags & BodyItem::LinkSetUpdate) &&
                        (treeWidget.isLinkItemVisible() || treeWidget.isJointItemVisible())) ||
                       ((flags & BodyItem::DeviceSetUpdate) && treeWidget.isDeviceItemVisible())){
                        treeWidget.updateTreeItems();
                    }
                });
    }
}


void LinkDeviceListView::Impl::onTreeWidgetLinkSelectionChanged()
{
    bodySelectionManagerConnection.block();
    bodySelectionManager->setLinkSelection(
        treeWidget.bodyItem(), treeWidget.linkSelection());
    bodySelectionManagerConnection.unblock();
}


bool LinkDeviceListView::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool LinkDeviceListView::Impl::storeState(Archive& archive)
{
    const char* etype = nullptr;
    switch(elementTypeCombo.currentIndex()){
    case ALL:    etype = "all";    break;
    case LINK:   etype = "link";   break;
    case JOINT:  etype = "joint";  break;
    case DEVICE: etype = "device"; break;
    default: break;
    }
    if(etype){
        archive.write("element_type", etype);
    }
        
    const char* listingModeSymbol = nullptr;
    switch(listingModeCombo.currentIndex()){
    case LinkDeviceTreeWidget::List: listingModeSymbol = "list"; break;
    case LinkDeviceTreeWidget::Tree: listingModeSymbol = "tree"; break;
    case LinkDeviceTreeWidget::GroupedTree: listingModeSymbol = "grouped_tree"; break;
    default: break;
    }
    if(listingModeSymbol){
        archive.write("listing_mode", listingModeSymbol);
    }

    if(auto bodyItem = treeWidget.bodyItem()){
        archive.writeItemId("current_body_item", bodyItem);
    }
    return treeWidget.storeState(archive);
}


bool LinkDeviceListView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LinkDeviceListView::Impl::restoreState(const Archive& archive)
{
    string symbol;
    int etype = LINK;
    if(archive.read("element_type", symbol)){
        if(symbol == "all"){
            etype = ALL;
        } else if(symbol == "link"){
            etype = LINK;
        } else if(symbol == "joint"){
            etype = JOINT;
        } else if(symbol == "device"){
            etype = DEVICE;
        }
    }
    elementTypeCombo.blockSignals(true);
    elementTypeCombo.setCurrentIndex(etype);
    elementTypeCombo.blockSignals(false);

    int listingMode = LinkDeviceTreeWidget::List;
    if(archive.read("listing_mode", symbol)){
        if(symbol == "list"){
            listingMode = LinkDeviceTreeWidget::List;
        } else if(symbol == "tree"){
            listingMode = LinkDeviceTreeWidget::Tree;
        } else if(symbol == "grouped_tree"){
            listingMode = LinkDeviceTreeWidget::GroupedTree;
        }
    }
    listingModeCombo.blockSignals(true);
    listingModeCombo.setCurrentIndex(listingMode);
    listingModeCombo.blockSignals(false);

    needToInitializeTreeWidgetElementTypeAndListingMode = true;
    
    archive.addPostProcess(
        [this, &archive](){
            if(treeWidget.restoreState(archive)){
                auto bodyItem = archive.findItem<BodyItem>("current_body_item");
                setCurrentBodyItem(bodyItem, nullptr);
            }
        });

    return true;
}
