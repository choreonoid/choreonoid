#include "LinkDeviceListView.h"
#include "LinkDeviceTreeWidget.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/CheckBox>
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
    CheckBox treeModeCheck;
    ButtonGroup elementRadioGroup;
    RadioButton allRadio;
    RadioButton linkRadio;
    RadioButton deviceRadio;
    BodySelectionManager* bodySelectionManager;
    ScopedConnection bodySelectionManagerConnection;
    bool isTreeWidgetUpdateEnabled;

    Impl(LinkDeviceListView* self);
    void onTargetElementTypeChanged(int id);
    void onTreeModeToggled(bool on);
    void onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link);
    void onTreeWidgetLinkSelectionChanged();
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
    int lmargin = self->style()->pixelMetric(QStyle::PM_LayoutLeftMargin);
    hbox->addSpacing(lmargin / 2);
    allRadio.setText(_("All"));
    elementRadioGroup.addButton(&allRadio, 0);
    hbox->addWidget(&allRadio);
    linkRadio.setText(_("Links"));
    linkRadio.setChecked(true);
    elementRadioGroup.addButton(&linkRadio, 1);
    hbox->addWidget(&linkRadio);
    deviceRadio.setText(_("Devices"));
    elementRadioGroup.addButton(&deviceRadio, 2);
    hbox->addWidget(&deviceRadio);

    elementRadioGroup.sigButtonToggled().connect(
        [&](int id, bool checked){
            if(checked){ onTargetElementTypeChanged(id); }
        });
    
    hbox->addStretch();
    treeModeCheck.setText(_("Tree"));
    treeModeCheck.sigToggled().connect(
        [&](bool on){ onTreeModeToggled(on); });
    hbox->addWidget(&treeModeCheck);
    vbox->addLayout(hbox);

    auto hframe = new QFrame;
    hframe->setFrameStyle(QFrame::HLine | QFrame::Sunken);
    vbox->addWidget(hframe);

    treeWidget.setCacheEnabled(true);
    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setVerticalGridLineShown(true);
    vbox->addWidget(&treeWidget);
    
    self->setLayout(vbox);

    treeWidgetConnection =
        treeWidget.sigLinkSelectionChanged().connect(
            [&](){ onTreeWidgetLinkSelectionChanged(); });

    bodySelectionManager = BodySelectionManager::instance();

    isTreeWidgetUpdateEnabled = true;
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


void LinkDeviceListView::Impl::onTargetElementTypeChanged(int id)
{
    treeWidget.setLinkItemVisible(id != 2);
    treeWidget.setDeviceItemVisible(id != 1);
    if(isTreeWidgetUpdateEnabled){
        treeWidget.updateTreeItems();
    }
}


void LinkDeviceListView::Impl::onTreeModeToggled(bool on)
{
    if(on){
        treeWidget.setListingMode(LinkDeviceTreeWidget::Tree);

        if(deviceRadio.isChecked()){
            elementRadioGroup.blockSignals(true);
            allRadio.setChecked(true);
            treeWidget.setLinkItemVisible(true);
            treeWidget.setDeviceItemVisible(true);
            elementRadioGroup.blockSignals(false);
        }
        deviceRadio.setEnabled(false);
            
    } else {
        treeWidget.setListingMode(LinkDeviceTreeWidget::List);
        deviceRadio.setEnabled(true);
    }
    
    if(isTreeWidgetUpdateEnabled){
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
    archive.write("mode", impl->treeModeCheck.isChecked() ? "tree" : "list");

    const char* etype = nullptr;
    switch(impl->elementRadioGroup.checkedId()){
    case 0: etype = "all";    break;
    case 1: etype = "link";   break;
    case 2: etype = "device"; break;
    default: break;
    }
    archive.write("element_type", etype);
        
    if(auto bodyItem = impl->treeWidget.bodyItem()){
        archive.writeItemId("current_body_item", bodyItem);
    }
    return impl->treeWidget.storeState(archive);
}


bool LinkDeviceListView::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool LinkDeviceListView::Impl::restoreState(const Archive& archive)
{
    int mode = LinkDeviceTreeWidget::List;
    string symbol;
    if(archive.read("mode", symbol)){
        if(symbol == "tree"){
            mode = LinkDeviceTreeWidget::Tree;
        }
    }
    int etype = 1;
    if(archive.read("element_type", symbol)){
        if(symbol == "all"){
            etype = 0;
        } else if(symbol == "device"){
            etype = 2;
        }
    }
    
    isTreeWidgetUpdateEnabled = false;
    treeModeCheck.setChecked(mode == LinkDeviceTreeWidget::Tree);
    elementRadioGroup.button(etype)->setChecked(true);
    isTreeWidgetUpdateEnabled = true;

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
