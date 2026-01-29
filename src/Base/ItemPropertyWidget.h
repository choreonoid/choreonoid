#ifndef CNOID_BASE_ITEM_PROPERTY_WIDGET_H
#define CNOID_BASE_ITEM_PROPERTY_WIDGET_H

#include <QWidget>
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Item;
class PutPropertyFunction;
class MenuManager;

class CNOID_EXPORT ItemPropertyWidget : public QWidget
{
public:
    ItemPropertyWidget(QWidget* parent = nullptr);
    ~ItemPropertyWidget();

    template<class ItemType>
    void setPropertyFunction(std::function<void(ItemType* item, PutPropertyFunction& putProperty)> func){
        setPropertyFunction_(
            typeid(ItemType),
            [func](Item* item, PutPropertyFunction& putProperty){
                func(static_cast<ItemType*>(item), putProperty);
            });
    };

    template<class ItemType>
    void addPropertyFunction(std::function<void(ItemType* item, PutPropertyFunction& putProperty)> func){
        addPropertyFunction_(
            typeid(ItemType),
            [func](Item* item, PutPropertyFunction& putProperty){
                func(static_cast<ItemType*>(item), putProperty);
            });
    };

    bool hasPropertyFunctionFor(Item* item) const;
    bool hasAdditionalPropertyFunctionFor(Item* item) const;
    void dispatchAdditionalPropertyFunctions(Item* item);

    void setCurrentItem(Item* item);
    void updateProperties();
    void resetColumnSizes();
    void setOperationMenu(MenuManager& menuManager);

    class Impl;

protected:
    void keyPressEvent(QKeyEvent* event);

private:
    void setPropertyFunction_(
        const std::type_info& type, std::function<void(Item* item, PutPropertyFunction& putProperty)> func);
    void addPropertyFunction_(
        const std::type_info& type, std::function<void(Item* item, PutPropertyFunction& putProperty)> func);

    Impl* impl;

    friend class ItemPropertyView;
};

}

#endif
