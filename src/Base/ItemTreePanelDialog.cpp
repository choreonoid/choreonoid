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

namespace {

class PanelArea : public QWidget
{
public:
    vector<function<QSize()>> minimumPanelSizeHintFunctions;

    virtual QSize minimumSizeHint() const override
    {
        auto size = QWidget::minimumSizeHint();
        for(auto& sizeHintFunc : minimumPanelSizeHintFunctions){
            auto panelSize = sizeHintFunc();
            size = size.expandedTo(panelSize);
        }
        return size;
    }
};

}

namespace cnoid {

class ItemTreePanelDialog::Impl
{
public:
    ItemTreePanelDialog* self;
    ItemPtr topItem;
    ItemPtr currentItem;
    ScopedConnection currentItemConnection;
    int mode;
    bool isPanelOnlyDisplayMode;
    bool isLastValidPanelKeepingMode;
    bool isSinglePanelSyncMode;
    bool needToUpdatePanel;
    PolymorphicItemFunctionSet  panelFunctions;
    ItemTreePanelBase* panelToActivate;
    ItemTreePanelBase* currentPanel;
    QLabel topAreaLabel;
    QHBoxLayout topWidgetLayout;
    ItemTreeWidget itemTreeWidget;
    ScopedConnection itemTreeWidgetConnection;
    QFrame panelFrame;
    PanelArea panelArea;
    QVBoxLayout panelLayout;
    QLabel panelCaptionLabel;
    QLabel defaultPanelLabel;
    QRect lastDialogPosition;
    
    Impl(ItemTreePanelDialog* self);
    void initialize();
    ~Impl();
    void setPanelOnlyDisplayMode(bool on);
    void updateTopAreaLayout();
    void showPanel();
    void onSelectionChanged(const ItemList<>& items);
    void activateItem(Item* item, bool isNewItem, bool doKeepLastPanel);
    void deactivateCurrentPanel(bool doClearCurrentItem);
    void deactivatePanel(ItemTreePanelBase* panel, bool isPanelAccepted);
    void clear();
};

}


ItemTreePanelBase::ItemTreePanelBase(QWidget* parent, Qt::WindowFlags f)
    : QWidget(parent, f)
{
    currentDialog = nullptr;
}


bool ItemTreePanelBase::activate
(Item* topItem, Item* elementItem, bool isNewItem, ItemTreePanelDialog* currentDialog)
{
    this->currentDialog = currentDialog;
    return onActivated(topItem, elementItem, isNewItem);
}


void ItemTreePanelBase::accept()
{
    if(currentDialog){
        currentDialog->impl->deactivatePanel(this, true);
    } else {
        onDeactivated();
    }
}


void ItemTreePanelBase::reject()
{
    if(currentDialog){
        currentDialog->impl->deactivatePanel(this, false);
    } else {
        onDeactivated();
    }
}


void ItemTreePanelBase::onDeactivated()
{

}


ItemTreeWidget* ItemTreePanelBase::itemTreeWidget()
{
    if(currentDialog){
        return currentDialog->itemTreeWidget();
    }
    return nullptr;
}


ItemTreePanelDialog::ItemTreePanelDialog()
{
    impl = new Impl(this);
    impl->initialize();
}


ItemTreePanelDialog::ItemTreePanelDialog(QWidget* parent, Qt::WindowFlags f)
    : Dialog(parent, f)
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
    mode = 0;
    isPanelOnlyDisplayMode = false;
    isLastValidPanelKeepingMode = false;
    isSinglePanelSyncMode = false;
    needToUpdatePanel = true;
    panelToActivate = nullptr;
    currentPanel = nullptr;

    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    topAreaLabel.setText(_("Add : "));
    topAreaLabel.setStyleSheet("font-weight: bold");
    hbox->addWidget(&topAreaLabel);
    hbox->addLayout(&topWidgetLayout);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    itemTreeWidget.setDragDropEnabled(false);
    itemTreeWidget.setCheckColumnShown(false);
    QFontMetrics metrics(itemTreeWidget.font());
    int width = metrics.averageCharWidth() * 24;
    itemTreeWidget.setMinimumWidth(width);
    itemTreeWidget.setMaximumWidth(width);
    hbox->addWidget(&itemTreeWidget, 0);

    int mh = self->style()->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int mv = self->style()->pixelMetric(QStyle::PM_LayoutTopMargin);

    panelFrame.setFrameStyle(QFrame::StyledPanel);
    panelFrame.setLineWidth(2);
    auto frameLayout = new QVBoxLayout;
    frameLayout->setContentsMargins(0, mh / 2, 0, 0);
    frameLayout->setSpacing(0);
    panelCaptionLabel.setStyleSheet("font-weight: bold");
    panelCaptionLabel.setAlignment(Qt::AlignHCenter);
    frameLayout->addWidget(&panelCaptionLabel);
    frameLayout->addWidget(&panelArea);
    panelFrame.setLayout(frameLayout);

    panelArea.setLayout(&panelLayout);
    panelLayout.setContentsMargins(0, 0, 0, 0);
    defaultPanelLabel.setAlignment(Qt::AlignCenter);
    defaultPanelLabel.setContentsMargins(mh * 2, mv, mh * 2, mv);
    panelLayout.addWidget(&defaultPanelLabel, Qt::AlignCenter);

    hbox->addWidget(&panelFrame, 1);
    vbox->addLayout(hbox);

    itemTreeWidget.customizeVisibility<Item>(
        [](Item*, bool){ return false; });

    itemTreeWidgetConnection =
        itemTreeWidget.sigSelectionChanged().connect(
            [&](const ItemList<>& items){ onSelectionChanged(items); });
}


ItemTreePanelDialog::~ItemTreePanelDialog()
{
    delete impl;
}


ItemTreePanelDialog::Impl::~Impl()
{

}


void ItemTreePanelDialog::setMode(int flags)
{
    impl->mode = flags;
    impl->setPanelOnlyDisplayMode(flags & PanelOnlyDisplayMode);
    impl->isLastValidPanelKeepingMode = flags & LastValidPanelKeepingMode;
    impl->isSinglePanelSyncMode = flags & SinglePanelSyncMode;
}


int ItemTreePanelDialog::mode() const
{
    return impl->mode;
}
    

void ItemTreePanelDialog::Impl::setPanelOnlyDisplayMode(bool on)
{
    if(on != isPanelOnlyDisplayMode){
        topAreaLabel.setVisible(!on);
        itemTreeWidget.setVisible(!on);
        panelFrame.setFrameStyle(on ? QFrame::NoFrame : QFrame::StyledPanel);
        panelCaptionLabel.setVisible(!on);
        isPanelOnlyDisplayMode = on;
    }
}


ItemTreeWidget* ItemTreePanelDialog::itemTreeWidget()
{
    return &impl->itemTreeWidget;
}


void ItemTreePanelDialog::registerPanel_
(const std::type_info& type,
 const std::function<ItemTreePanelBase*(Item* item)>& panelFunction,
 const std::function<QSize()>& minimumSizeHintFunction)
{
    impl->panelFunctions.setFunction(
        type, [this, panelFunction](Item* item){ impl->panelToActivate = panelFunction(item); });

    impl->panelArea.minimumPanelSizeHintFunctions.push_back(minimumSizeHintFunction);
}


void ItemTreePanelDialog::addTopAreaWidget(QWidget* widget)
{
    impl->topWidgetLayout.addWidget(widget);
}


void ItemTreePanelDialog::updateTopAreaLayout()
{
    impl->updateTopAreaLayout();
}


void ItemTreePanelDialog::Impl::updateTopAreaLayout()
{
    bool hasVisibleWidget = false;
    int n = topWidgetLayout.count();
    for(int i=0; i < n; ++i){
        auto item = topWidgetLayout.itemAt(i);
        if(auto widget = item->widget()){
            if(!widget->isHidden()){
                hasVisibleWidget = true;
                break;
            }
        }
    }
    topAreaLabel.setVisible(hasVisibleWidget);
}


bool ItemTreePanelDialog::setTopItem(Item* item, bool isTopVisible)
{
    impl->topItem = item;
    impl->itemTreeWidget.setRootItemVisible(isTopVisible);
    impl->itemTreeWidget.setRootItem(item);
    impl->needToUpdatePanel = true;
    return true;
}


void ItemTreePanelDialog::show()
{
    impl->showPanel();

    bool isPanelCaptionHidden = false;

    if(!impl->isPanelOnlyDisplayMode){
        // The label must be displayed when the dialog is shown
        // to ensure the necessary dialog size
        isPanelCaptionHidden = impl->panelCaptionLabel.isHidden();
        impl->panelCaptionLabel.show();
    }
    
    Dialog::show();

    // Restore the label visibility
    if(isPanelCaptionHidden){
        impl->panelCaptionLabel.hide();
    }
    
    if(!impl->lastDialogPosition.isNull()){
        setGeometry(impl->lastDialogPosition);
    }
}


void ItemTreePanelDialog::Impl::showPanel()
{
    auto items = itemTreeWidget.getSelectedItems();
    if(items.empty()){
        items = itemTreeWidget.getItems();
    }
    onSelectionChanged(items);
}


bool ItemTreePanelDialog::setCurrentItem(Item* item, bool isNewItem)
{
    impl->itemTreeWidgetConnection.block();
    bool selected = impl->itemTreeWidget.selectOnly(item);
    impl->itemTreeWidgetConnection.unblock();

    if(selected){
        impl->activateItem(item, isNewItem, impl->isLastValidPanelKeepingMode);
    }

    return selected;
}



void ItemTreePanelDialog::Impl::onSelectionChanged(const ItemList<>& items)
{
    Item* item = nullptr;
    if(!items.empty()){
        item = items.front();
    }
    if(!isLastValidPanelKeepingMode || item){
        activateItem(item, false, isLastValidPanelKeepingMode);
    }
}


void ItemTreePanelDialog::Impl::activateItem(Item* item, bool isNewItem, bool doKeepLastPanel)
{
    if(item == currentItem && !needToUpdatePanel){
        return;
    }

    bool updated = false;

    panelToActivate = nullptr;
    if(item){
        panelFunctions.dispatch(item);
    }

    if(!doKeepLastPanel || panelToActivate){
        deactivateCurrentPanel(false);
        currentItem = item;
        if(currentItem){
            currentItemConnection =
                currentItem->sigDisconnectedFromRoot().connect(
                    [this](){ deactivateCurrentPanel(true); });
        } else {
            currentItemConnection.disconnect();
        }
        panelCaptionLabel.setText(_("Unkown"));
        updated = true;
    }

    if(!item){
        if(!doKeepLastPanel){
            panelCaptionLabel.hide();
            if(itemTreeWidget.getItems().empty()){
                defaultPanelLabel.setText(_("There are no items to be configured."));
            } else {
                defaultPanelLabel.setText(_("No item is selected."));
            }
            defaultPanelLabel.show();
        }
    } else {
        if(panelToActivate){
            if(panelToActivate->activate(topItem, item, isNewItem, self)){
                currentPanel = panelToActivate;
                defaultPanelLabel.hide();
                if(!isPanelOnlyDisplayMode){
                    panelCaptionLabel.setText(currentPanel->caption().c_str());
                    panelCaptionLabel.show();
                }
                panelLayout.addWidget(currentPanel);
                currentPanel->show();
            }
            panelToActivate = nullptr;

        } else if(!doKeepLastPanel){
            if(!isPanelOnlyDisplayMode){
                panelCaptionLabel.show();
            }
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

    if(updated){
        self->onCurrentItemChanged(currentItem);
    }
}


void ItemTreePanelDialog::onCurrentItemChanged(Item*)
{

}


void ItemTreePanelDialog::Impl::deactivateCurrentPanel(bool doClearCurrentItem)
{
    if(currentPanel){
        currentPanel->onDeactivated();
        panelLayout.removeWidget(currentPanel);
        currentPanel->hide();
        currentPanel = nullptr;
    }
    needToUpdatePanel = true;
    
    if(currentItem && doClearCurrentItem){
        itemTreeWidgetConnection.block();
        currentItem->setSelected(false);
        currentItem.reset();
        currentItemConnection.disconnect();
        itemTreeWidgetConnection.unblock();
        activateItem(nullptr, false, false);
    }
}


void ItemTreePanelDialog::Impl::deactivatePanel(ItemTreePanelBase* panel, bool isPanelAccepted)
{
    int numItems = itemTreeWidget.getItems().size();
    
    if(panel == currentPanel){
        deactivateCurrentPanel(!isPanelAccepted);
    }

    if(isSinglePanelSyncMode){
        self->hide();
    } else if(numItems <= 1){
        self->hide();
    } else if(isPanelAccepted){
        showPanel();
    }
}


void ItemTreePanelDialog::Impl::clear()
{
    itemTreeWidget.setRootItem(nullptr);
    deactivateCurrentPanel(true);
    topItem.reset();
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
