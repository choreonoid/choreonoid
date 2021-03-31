#include "LinkDeviceListView.h"
#include "LinkDeviceTreeWidget.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
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
    ScopedConnection treeWidgetConnection;
    enum ElementType { ALL, LINK, JOINT, DEVICE };
    ComboBox elementTypeCombo;
    ComboBox listingModeCombo;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionManagerConnection;

    Impl(LinkDeviceListView* self);
    void onElementTypeChanged(int id, bool doUpdate);
    void onListingModeChanged(int id, bool doUpdate);
    void onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link);
    void onTreeWidgetLinkSelectionChanged();
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
};

}


void LinkDeviceListView::initializeClass(ExtensionManager* ext)
{
    auto& vm = ext->viewManager();
    vm.registerClass<LinkDeviceListView>(
        "LinkDeviceListView", N_("Links / Devices"), ViewManager::SINGLE_OPTIONAL);
    vm.registerClassAlias("LinkSelectionView", "Body::LinkDeviceListView");
}


LinkDeviceListView::LinkDeviceListView()
{
    impl = new Impl(this);
}


LinkDeviceListView::Impl::Impl(LinkDeviceListView* self)
{
    self->setDefaultLayoutArea(View::LEFT_BOTTOM);
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
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);

    treeWidget.setCacheEnabled(true);
    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setVerticalGridLineShown(true);
    vbox->addWidget(&treeWidget);
    
    self->setLayout(vbox);

    elementTypeCombo.sigCurrentIndexChanged().connect(
        [&](int id){ onElementTypeChanged(id, true); });

    listingModeCombo.sigCurrentIndexChanged().connect(
        [&](int id){ onListingModeChanged(id, true); });

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
                impl->onCurrentBodySelectionChanged(bodyItem, link); });
    
    impl->onCurrentBodySelectionChanged(bsm->currentBodyItem(), bsm->currentLink());
}


void LinkDeviceListView::onDeactivated()
{
    impl->bodySelectionManagerConnection.disconnect();
}


void LinkDeviceListView::Impl::onElementTypeChanged(int id, bool doUpdate)
{
    if(id == DEVICE && listingModeCombo.currentIndex() != LinkDeviceTreeWidget::List){
        listingModeCombo.blockSignals(true);
        listingModeCombo.setCurrentIndex(LinkDeviceTreeWidget::List);
        onListingModeChanged(LinkDeviceTreeWidget::List, false);
        listingModeCombo.blockSignals(false);
    }
    
    treeWidget.setLinkItemVisible(id == ALL || id == LINK);
    treeWidget.setJointItemVisible(id == JOINT);
    treeWidget.setDeviceItemVisible(id == ALL || id == DEVICE);

    if(doUpdate){
        treeWidget.updateTreeItems();
    }
}


void LinkDeviceListView::Impl::onListingModeChanged(int id, bool doUpdate)
{
    treeWidget.setListingMode(id);

    if(id != LinkDeviceTreeWidget::List){
        if(elementTypeCombo.currentIndex() == DEVICE){
            elementTypeCombo.blockSignals(true);
            elementTypeCombo.setCurrentIndex(ALL);
            treeWidget.setLinkItemVisible(true);
            treeWidget.setDeviceItemVisible(true);
            onElementTypeChanged(ALL, false);
            elementTypeCombo.blockSignals(false);
        }
    }

    if(doUpdate){
        treeWidget.updateTreeItems();
    }
}    


void LinkDeviceListView::Impl::onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link)
{
    if(bodyItem && link){
        treeWidget.setLinkSelection(bodyItem, bodySelectionManager->linkSelection(bodyItem));
    }
    treeWidget.setBodyItem(bodyItem);
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
        archive.write("listingMode", listingModeSymbol);
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
    if(archive.read("mode", symbol)){
        if(symbol == "list"){
            listingMode = LinkDeviceTreeWidget::List;
        }
        if(symbol == "tree"){
            listingMode = LinkDeviceTreeWidget::Tree;
        }
        if(symbol == "grouped_tree"){
            listingMode = LinkDeviceTreeWidget::GroupedTree;
        }
    }
    listingModeCombo.blockSignals(true);
    listingModeCombo.setCurrentIndex(listingMode);
    listingModeCombo.blockSignals(false);
    
    archive.addPostProcess(
        [this, &archive](){
            if(treeWidget.restoreState(archive)){
                if(auto item = archive.findItem<BodyItem>("current_body_item")){
                    treeWidget.setBodyItem(item, true);
                } else {
                    treeWidget.updateTreeItems();
                }
            }
        });

    return true;
}
