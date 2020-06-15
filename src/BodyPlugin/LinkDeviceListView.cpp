#include "LinkDeviceListView.h"
#include "LinkDeviceTreeWidget.h"
#include "BodyItem.h"
#include "BodySelectionManager.h"
#include <cnoid/Link>
#include <cnoid/ViewManager>
#include <cnoid/Buttons>
#include <cnoid/ButtonGroup>
#include <cnoid/CheckBox>
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
    ext->viewManager().registerClass<LinkDeviceListView>(
        "LinkDeviceListView", N_("Links / Devices"), ViewManager::SINGLE_OPTIONAL);
}


LinkDeviceListView* LinkDeviceListView::instance()
{
    static LinkDeviceListView* instance_ = ViewManager::getOrCreateView<LinkDeviceListView>();
    return instance_;
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

    treeWidget.setFrameShape(QFrame::NoFrame);
    treeWidget.setMode(LinkDeviceTreeWidget::List);
    treeWidget.setLinkItemVisible(true);
    treeWidget.setDeviceItemVisible(false);
    treeWidget.setCacheEnabled(true);
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

    isTreeWidgetUpdateEnabled = true;
}


LinkDeviceListView::~LinkDeviceListView()
{
    delete impl;
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
        treeWidget.setMode(LinkDeviceTreeWidget::Tree);

        if(deviceRadio.isChecked()){
            elementRadioGroup.blockSignals(true);
            allRadio.setChecked(true);
            treeWidget.setLinkItemVisible(true);
            treeWidget.setDeviceItemVisible(true);
            elementRadioGroup.blockSignals(false);
        }
        deviceRadio.setEnabled(false);
            
    } else {
        treeWidget.setMode(LinkDeviceTreeWidget::List);
        deviceRadio.setEnabled(true);
    }
    
    if(isTreeWidgetUpdateEnabled){
        treeWidget.updateTreeItems();
    }
}    


BodyItem* LinkDeviceListView::currentBodyItem()
{
    return impl->treeWidget.bodyItem();
}


SignalProxy<void(int linkIndex)> LinkDeviceListView::sigCurrentLinkChanged()
{
    return impl->treeWidget.sigCurrentLinkChanged();
}


int LinkDeviceListView::currentLinkIndex() const
{
    return impl->treeWidget.currentLinkIndex();
}


void LinkDeviceListView::setCurrentLink(int index)
{
    impl->treeWidget.setCurrentLink(index);
}


SignalProxy<void()> LinkDeviceListView::sigLinkSelectionChanged()
{
    return impl->treeWidget.sigLinkSelectionChanged();
}


const std::vector<int>& LinkDeviceListView::selectedLinkIndices() const
{
    return impl->treeWidget.selectedLinkIndices();
}


const std::vector<bool>& LinkDeviceListView::linkSelection() const
{
    return impl->treeWidget.linkSelection();
}


void LinkDeviceListView::Impl::onCurrentBodySelectionChanged(BodyItem* bodyItem, Link* link)
{
    treeWidget.setBodyItem(bodyItem);
    if(bodyItem && link){
        treeWidget.setCurrentLink(link->index());
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

    if(treeWidget.restoreState(archive)){
        archive.addPostProcess(
            [this, &archive](){
                if(auto item = archive.findItem<BodyItem>("current_body_item")){
                    treeWidget.setBodyItem(item);
                } else {
                    treeWidget.updateTreeItems();
                }
            });
        return true;
    }

    return false;
}
