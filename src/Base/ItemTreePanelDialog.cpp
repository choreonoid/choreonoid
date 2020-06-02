#include "ItemTreePanelDialog.h"
#include "Item.h"
#include "ItemTreeWidget.h"
#include <cnoid/PolymorphicItemFunctionSet>
#include <QBoxLayout>
#include <QLabel>
#include <QStyle>
#include <QKeyEvent>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class ItemTreePanelDialog::Impl
{
public:
    ItemTreePanelDialog* self;
    ItemPtr topItem;
    ItemPtr currentItem;
    bool needToUpdatePanel;
    PolymorphicItemFunctionSet  panelFunctions;
    ItemTreePanelBase* panelToActivate;
    ItemTreePanelBase* currentPanel;
    QHBoxLayout topWidgetLayout;
    ItemTreeWidget itemTreeWidget;
    ScopedConnection itemTreeWidgetConnection;
    QFrame panelFrame;
    QVBoxLayout panelLayout;
    QLabel panelCaptionLabel;
    QLabel defaultPanelLabel;
    QRect lastDialogPosition;
    
    Impl(ItemTreePanelDialog* self);
    void initialize();
    ~Impl();
    void onSelectionChanged(const ItemList<>& items);
    void activateItem(Item* item);
    void deactivateCurrentPanel();
    void deactivatePanel(ItemTreePanelBase* panel);
    void clear();
};

}


ItemTreePanelBase::ItemTreePanelBase(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f)
{
    currentDialogImpl = nullptr;
}


bool ItemTreePanelBase::activate
(Item* topItem, Item* elementItem, bool isNewItem, ItemTreePanelDialog::Impl* currentDialogImpl)
{
    this->currentDialogImpl = currentDialogImpl;
    return onActivated(topItem, elementItem, isNewItem);
}


void ItemTreePanelBase::deactivate()
{
    if(currentDialogImpl){
        currentDialogImpl->deactivatePanel(this);
    } else {
        onDeactivated();
    }
}


void ItemTreePanelBase::onDeactivated()
{

}


ItemTreePanelDialog::ItemTreePanelDialog()
{
    impl = new Impl(this);
    impl->initialize();
}


ItemTreePanelDialog::Impl::Impl(ItemTreePanelDialog* self)
    : self(self)
{

}


void ItemTreePanelDialog::Impl::initialize()
{
    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    auto addLabel = new QLabel(_("Add : "));
    addLabel->setStyleSheet("font-weight: bold");
    hbox->addWidget(addLabel);
    hbox->addLayout(&topWidgetLayout);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    itemTreeWidget.setDragDropEnabled(false);
    itemTreeWidget.setCheckColumnShown(false);
    hbox->addWidget(&itemTreeWidget);

    int mh = self->style()->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int mv = self->style()->pixelMetric(QStyle::PM_LayoutTopMargin);

    panelFrame.setFrameStyle(QFrame::StyledPanel);
    panelFrame.setLineWidth(2);
    panelFrame.setLayout(&panelLayout);
    panelCaptionLabel.setStyleSheet("font-weight: bold");
    panelCaptionLabel.setAlignment(Qt::AlignHCenter);
    panelLayout.addWidget(&panelCaptionLabel);
    panelLayout.setContentsMargins(0, mh / 2, 0, 0);
    panelLayout.setSpacing(0);
    defaultPanelLabel.setAlignment(Qt::AlignCenter);
    defaultPanelLabel.setContentsMargins(mh * 2, mv, mh * 2, mv);
    panelLayout.addWidget(&defaultPanelLabel, Qt::AlignCenter);

    hbox->addWidget(&panelFrame);

    vbox->addLayout(hbox);

    itemTreeWidget.customizeVisibility<Item>(
        [](Item*, bool){ return false; });

    itemTreeWidgetConnection =
        itemTreeWidget.sigSelectionChanged().connect(
            [&](const ItemList<>& items){ onSelectionChanged(items); });

    needToUpdatePanel = true;
    panelToActivate = nullptr;
    currentPanel = nullptr;
}


ItemTreePanelDialog::~ItemTreePanelDialog()
{
    delete impl;
}


ItemTreePanelDialog::Impl::~Impl()
{

}


ItemTreeWidget* ItemTreePanelDialog::itemTreeWidget()
{
    return &impl->itemTreeWidget;
}


void ItemTreePanelDialog::registerPanel_
(const std::type_info& type, std::function<ItemTreePanelBase*(Item* item)> panelFunction)
{
    impl->panelFunctions.setFunction(
        type, [this, panelFunction](Item* item){ impl->panelToActivate = panelFunction(item); });
}


void ItemTreePanelDialog::addTopAreaWidget(QWidget* widget)
{
    impl->topWidgetLayout.addWidget(widget);
}


bool ItemTreePanelDialog::setTopItem(Item* item)
{
    impl->topItem = item;
    impl->itemTreeWidget.setRootItem(item);
    impl->needToUpdatePanel = true;
    return true;
}


void ItemTreePanelDialog::show()
{
    auto items = impl->itemTreeWidget.getSelectedItems();
    if(items.empty()){
        items = impl->itemTreeWidget.getItems();
    }
    impl->onSelectionChanged(items);
    
    Dialog::show();
    
    if(!impl->lastDialogPosition.isNull()){
        setGeometry(impl->lastDialogPosition);
    }
}


bool ItemTreePanelDialog::setCurrentItem(Item* item)
{
    return impl->itemTreeWidget.selectOnly(item);
}



void ItemTreePanelDialog::Impl::onSelectionChanged(const ItemList<>& items)
{
    Item* item = nullptr;
    if(!items.empty()){
        item = items.front();
    }
    activateItem(item);
}


void ItemTreePanelDialog::Impl::activateItem(Item* item)
{
    if(item == currentItem && !needToUpdatePanel){
        return;
    }
    deactivateCurrentPanel();
    currentItem = item;

    if(!item){
        panelCaptionLabel.hide();
        if(itemTreeWidget.getItems().empty()){
            defaultPanelLabel.setText(_("There are no items to be configured."));
        } else {
            defaultPanelLabel.setText(_("No item is selected."));
        }
        defaultPanelLabel.show();
    } else {
        panelToActivate = nullptr;
        panelFunctions.dispatch(item);
        if(panelToActivate){
            if(panelToActivate->activate(topItem, item, false, this)){
                currentPanel = panelToActivate;
                defaultPanelLabel.hide();
                panelCaptionLabel.setText(currentPanel->caption().c_str());
                panelCaptionLabel.show();
                panelLayout.addWidget(currentPanel);
                currentPanel->show();
            }
            panelToActivate = nullptr;
        } else {
            panelCaptionLabel.setText(_("Unkown"));
            panelCaptionLabel.show();
            defaultPanelLabel.show();
            defaultPanelLabel.setText(
                format(_("\"{0}\" is not a target item."), item->name()).c_str());
        }
        if(!item->isSelected()){
            itemTreeWidgetConnection.block();
            item->setSelected(true);
            itemTreeWidgetConnection.unblock();
        }
    }
    needToUpdatePanel = false;
}


void ItemTreePanelDialog::Impl::deactivateCurrentPanel()
{
    if(currentPanel){
        currentPanel->onDeactivated();
        panelLayout.removeWidget(currentPanel);
        currentPanel->hide();
        currentPanel = nullptr;
    }
    currentItem.reset();
}


void ItemTreePanelDialog::Impl::deactivatePanel(ItemTreePanelBase* panel)
{
    if(panel == currentPanel){
        deactivateCurrentPanel();
        needToUpdatePanel = true;
    }
    //if(numItems == 0) closeDialog
    self->show();
}


void ItemTreePanelDialog::Impl::clear()
{
    itemTreeWidget.setRootItem(nullptr);
    deactivateCurrentPanel();
    topItem.reset();
    currentItem.reset();
}


void ItemTreePanelDialog::keyPressEvent(QKeyEvent* event)
{
    int key = event->key();
    // Prevent the dialog from closing when the enter key is pressed on a child widget
    if(key == Qt::Key_Return || key == Qt::Key_Enter){
        event->ignore();
    } else {
        QDialog::keyPressEvent(event);
    }
}


void ItemTreePanelDialog::hideEvent(QHideEvent* event)
{
    impl->clear();
    impl->lastDialogPosition = geometry();
    Dialog::hideEvent(event);
}
