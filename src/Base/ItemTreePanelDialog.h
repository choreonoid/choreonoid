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
    ~ItemTreePanelDialog();

    ItemTreeWidget* itemTreeWidget();
    void addTopAreaWidget(QWidget* widget);
    
    template<class TargetItemType>
    void registerPanel(ItemTreePanelBase* panel){
        registerPanel_(typeid(TargetItemType), [panel](Item*){ return panel; });
    }

    template<class TargetItemType>
    void registerPanel(std::function<ItemTreePanelBase*(TargetItemType* item)> panelFunction){
        registerPanel_(
            typeid(TargetItemType),
            [panelFunction](Item* item){ return panelFunction(static_cast<TargetItemType*>(item)); });
    }

    bool setTopItem(Item* topItem);
    void show();
    bool setCurrentItem(Item* item);

    class Impl;

protected:
    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void hideEvent(QHideEvent* event) override;

private:
    Impl* impl;

    void registerPanel_(
        const std::type_info& type, std::function<ItemTreePanelBase*(Item* item)> panelFunction);
};

class CNOID_EXPORT ItemTreePanelBase : public QWidget
{
public:
    ItemTreePanelBase(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    bool activate(Item* topItem, Item* targetItem, bool isNewItem, ItemTreePanelDialog::Impl* currentDialogImpl);
    void deactivate();
    virtual std::string caption() const = 0;
    virtual bool onActivated(Item* topItem, Item* targetItem, bool isNewItem) = 0;
    virtual void onDeactivated();
private:
    ItemTreePanelDialog::Impl* currentDialogImpl;
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

}

#endif
