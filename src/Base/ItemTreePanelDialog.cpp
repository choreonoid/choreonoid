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
    QHBoxLayout topWidgetLayout;
    QHBoxLayout topAreaStretch;
    ItemTreeWidget itemTreeWidget;
    ScopedConnection itemTreeWidgetConnection;
    QFrame panelFrame;
    PanelArea panelArea;
    QVBoxLayout panelLayout;
    QWidget panelCaptionBase;
    QLabel panelCaptionLabel;
    QLabel defaultPanelLabel;
    
    Impl(ItemTreePanelDialog* self);
    void initialize();
    ~Impl();
    void setPanelOnlyDisplayMode(bool on);
    void showPanel();
    void onSelectionChanged(const ItemList<>& items);
    void activateItem(Item* item, bool isNewItem, bool doKeepLastPanel);
    void deactivateCurrentPanel(int deactivationType, bool doClearCurrentItem);
    void deactivatePanel(ItemTreePanelBase* panel, int deactivationType, bool doClearCurrentItem);
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


void ItemTreePanelBase::setCaption(const std::string& caption)
{
    if(caption != caption_){
        caption_ = caption;
        if(currentDialog){
            currentDialog->onCurrentPanelCaptionChanged();
        }
    }
}


void ItemTreePanelBase::accept()
{
    finish(Accepted);
}


void ItemTreePanelBase::reject()
{
    finish(Rejected);
}


void ItemTreePanelBase::finish(int deactivationType)
{
    if(!currentDialog){
        onDeactivated(deactivationType);
    } else {
        auto impl = currentDialog->impl;
        int numItems = impl->itemTreeWidget.getItems().size();
        impl->deactivatePanel(this, deactivationType, true);
        if(numItems <= 1){
            currentDialog->close();
        } else {
            impl->showPanel();
        }
    }
}


void ItemTreePanelBase::onDeactivated(int /* deactivationType */)
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
    self->setEnterKeyClosePreventionMode(true);
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

    auto topVBox = new QVBoxLayout;
    self->setLayout(topVBox);

    auto hbox = new QHBoxLayout;
    hbox->addLayout(&topWidgetLayout);
    topAreaStretch.addStretch();
    hbox->addLayout(&topAreaStretch);
    topVBox->addLayout(hbox);

    hbox = new QHBoxLayout;
    itemTreeWidget.setDragDropEnabled(false);
    itemTreeWidget.setCheckColumnShown(false);
    /*
    QFontMetrics metrics(itemTreeWidget.font());
    int width = metrics.averageCharWidth() * 24;
    itemTreeWidget.setMinimumWidth(width);
    */
    hbox->addWidget(&itemTreeWidget, 1);

    int mh = self->style()->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int mv = self->style()->pixelMetric(QStyle::PM_LayoutTopMargin);

    panelFrame.setFrameStyle(QFrame::StyledPanel);
    panelFrame.setLineWidth(2);
    auto frameLayout = new QVBoxLayout;
    frameLayout->setContentsMargins(0, 0, 0, 0);
    frameLayout->setSpacing(0);

    auto vbox = new QVBoxLayout;
    panelCaptionBase.setLayout(vbox);
    vbox->setContentsMargins(0, mv / 2, 0, 0);
    panelCaptionLabel.setStyleSheet("font-weight: bold");
    panelCaptionLabel.setAlignment(Qt::AlignHCenter);
    vbox->addWidget(&panelCaptionLabel);
    frameLayout->addWidget(&panelCaptionBase);
    
    frameLayout->addWidget(&panelArea);
    panelFrame.setLayout(frameLayout);

    panelArea.setLayout(&panelLayout);
    panelLayout.setContentsMargins(0, 0, 0, 0);
    defaultPanelLabel.setAlignment(Qt::AlignCenter);
    defaultPanelLabel.setContentsMargins(mh * 2, mv, mh * 2, mv);
    panelLayout.addWidget(&defaultPanelLabel, Qt::AlignCenter);

    hbox->addWidget(&panelFrame, 1);
    topVBox->addLayout(hbox);

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
        itemTreeWidget.setVisible(!on);
        panelFrame.setFrameStyle(on ? QFrame::NoFrame : QFrame::StyledPanel);
        panelCaptionBase.setVisible(!on);
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


void ItemTreePanelDialog::addTopAreaSpacing(int spacing)
{
    impl->topWidgetLayout.addSpacing(spacing);
}


void ItemTreePanelDialog::setTopAreaLayoutStretchEnabled(bool on)
{
    if(on){
        if(impl->topAreaStretch.count() == 0){
            impl->topAreaStretch.addStretch();
        }
    } else {
        impl->topAreaStretch.takeAt(0);
    }
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
        isPanelCaptionHidden = impl->panelCaptionBase.isHidden();
        impl->panelCaptionBase.show();
    }

    Dialog::show();

    // Restore the label visibility
    if(isPanelCaptionHidden){
        impl->panelCaptionBase.hide();
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


Item* ItemTreePanelDialog::currentItem()
{
    return impl->currentItem;
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
        deactivateCurrentPanel(ItemTreePanelBase::Undetermined, false);
        currentItem = item;
        if(currentItem){
            currentItemConnection =
                currentItem->sigDisconnectedFromRoot().connect(
                    [this](){ deactivateCurrentPanel(ItemTreePanelBase::Rejected, true); });
        } else {
            currentItemConnection.disconnect();
        }
        panelCaptionLabel.setText(_("Unkown"));
        updated = true;
    }

    if(!item){
        if(!doKeepLastPanel){
            panelCaptionBase.hide();
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
                    panelCaptionBase.show();
                }
                panelLayout.addWidget(currentPanel);
                currentPanel->show();
            }
            panelToActivate = nullptr;

        } else if(!doKeepLastPanel){
            if(!isPanelOnlyDisplayMode){
                panelCaptionBase.show();
            }
            defaultPanelLabel.show();
            defaultPanelLabel.setText(
                format(_("\"{0}\" is not a target item."), item->name()).c_str());
        }
        if(!item->isSelected()){
            itemTreeWidgetConnection.block();
            
            // Use the ItemTreeWidget::selectOnly function instead of Item::setSelected
            // to unselect other items in the sub tree
            itemTreeWidget.selectOnly(item);
            
            itemTreeWidgetConnection.unblock();
        }
    }
    needToUpdatePanel = false;

    if(updated){
        self->onCurrentItemChanged(currentItem);
    }
}


void ItemTreePanelDialog::onCurrentPanelCaptionChanged()
{
    if(impl->currentPanel){
        impl->panelCaptionLabel.setText(impl->currentPanel->caption().c_str());
    }
}


void ItemTreePanelDialog::onCurrentItemChanged(Item*)
{

}


void ItemTreePanelDialog::Impl::deactivateCurrentPanel(int deactivationType, bool doClearCurrentItem)
{
    deactivatePanel(currentPanel, deactivationType, doClearCurrentItem);
}


void ItemTreePanelDialog::Impl::deactivatePanel(ItemTreePanelBase* panel, int deactivationType, bool doClearCurrentItem)
{
    auto prevPanel = currentPanel;
    
    if(panel == currentPanel){
        if(panel){
            currentItemConnection.block();
            panel->onDeactivated(deactivationType);
            currentItemConnection.unblock();
            
            panelLayout.removeWidget(panel);
            panel->hide();
            currentPanel = nullptr;
        }

        /**
           It has been reported that the following flag update caueses a bug in the cooperation
           between item tree views and the panel dialog based on this ItemTreePanelDialog class.
           The bug seems to be a symptom that when some item is selected in another item tree view,
           an item in the panel dialog is unintentionally activated and then not deactivated.
           The bug may have been resolved by fixes made to other classes after the report, but
           if you encounter the bug, please check the following flag operation to resolve the problem.
           Disabling the flag update may solve the problem, but since the update is originally
           required for the dialog to work correctly, you may need another solution.
        */
        needToUpdatePanel = true;
    }

    if(currentItem && doClearCurrentItem){
        currentItemConnection.disconnect();
        currentItem.reset();
        activateItem(nullptr, false, false);
    }

    if(isSinglePanelSyncMode && prevPanel){
        self->close();
    }
}


void ItemTreePanelDialog::Impl::clear()
{
    itemTreeWidget.setRootItem(nullptr);
    deactivateCurrentPanel(ItemTreePanelBase::Undetermined, true);
    topItem.reset();
}


void ItemTreePanelDialog::closeEvent(QCloseEvent* event)
{
    if(onDialogClosed()){
        event->accept();
        impl->clear();
    } else {
        event->ignore();
    }
}


bool ItemTreePanelDialog::onDialogClosed()
{
    return true;
}
