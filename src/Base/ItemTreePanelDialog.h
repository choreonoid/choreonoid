#ifndef CNOID_BASE_ITEM_TREE_PANEL_DIALOG_H
#define CNOID_BASE_ITEM_TREE_PANEL_DIALOG_H

#include <cnoid/Dialog>
#include "exportdecl.h"

namespace cnoid {

class ItemTreeWidget;
class Item;
class ItemTreePanelBase;

class CNOID_EXPORT ItemTreePanelDialog : public Dialog
{
public:
    ItemTreePanelDialog();
    ItemTreePanelDialog(QWidget* parent, Qt::WindowFlags f= Qt::WindowFlags());
    ~ItemTreePanelDialog();

    enum ModeFlags {
        PanelOnlyDisplayMode = 1,
        LastValidPanelKeepingMode = 2,
        // The dialog is always closed when the operation on a single panel is accepted or rejected
        SinglePanelSyncMode = 4
    };

    void setMode(int flags);
    int mode() const;

    ItemTreeWidget* itemTreeWidget();

    template<class TargetItemType>
    void registerPanel(ItemTreePanelBase* panel);

    template<class TargetItemType>
    void registerPanel(
        std::function<ItemTreePanelBase*(TargetItemType* item)> panelFunction,
        std::function<QSize()> minimumSizeHintFunction)
    {
        registerPanel_(
            typeid(TargetItemType),
            [panelFunction](Item* item){ return panelFunction(static_cast<TargetItemType*>(item)); },
            minimumSizeHintFunction);
    }

    void addTopAreaWidget(QWidget* widget);
    void updateTopAreaLayout();
    
    bool setTopItem(Item* topItem, bool isTopVisible = false);
    void show();
    bool setCurrentItem(Item* item, bool isNewItem = false);

protected:
    virtual void onCurrentItemChanged(Item* item);
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void hideEvent(QHideEvent* event) override;

private:
    class Impl;
    Impl* impl;

    void registerPanel_(
        const std::type_info& type,
        const std::function<ItemTreePanelBase*(Item* item)>& panelFunction,
        const std::function<QSize()>& minimumSizeHintFunction);

    friend class ItemTreePanelBase;
        
};

class CNOID_EXPORT ItemTreePanelBase : public QWidget
{
public:
    ItemTreePanelBase(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    bool activate(Item* topItem, Item* targetItem, bool isNewItem, ItemTreePanelDialog* currentDialog);
    virtual std::string caption() const = 0;
    virtual bool onActivated(Item* topItem, Item* targetItem, bool isNewItem) = 0;
    virtual void onDeactivated();
    ItemTreePanelDialog* dialog(){ return currentDialog; }
    ItemTreeWidget* itemTreeWidget();

protected:
    void accept();
    void reject();
    
private:
    ItemTreePanelDialog* currentDialog;
};

template<class TopItemType, class TargetItemType>
class ItemTreePanel : public ItemTreePanelBase
{
public:
    ItemTreePanel(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags())
        : ItemTreePanelBase(parent, f)
    { }

    virtual bool onActivated(Item* topItem, Item* targetItem, bool isNewItem) override final {
        if(auto derivedTopItem = dynamic_cast<TopItemType*>(topItem)){
            if(auto derivedTargetItem = dynamic_cast<TargetItemType*>(targetItem)){
                return onActivated(derivedTopItem, derivedTargetItem, isNewItem);
            }
        }
        return false;
    }
    
    virtual bool onActivated(TopItemType* topItem, TargetItemType* targetItem, bool isNewItem) = 0;
};


template<class TargetItemType>
void ItemTreePanelDialog::registerPanel(ItemTreePanelBase* panel)
{
    registerPanel_(
        typeid(TargetItemType),
        [panel](Item*){ return panel; },
        [panel](){ return panel->minimumSizeHint(); });
}


}

#endif
