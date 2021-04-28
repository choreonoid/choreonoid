#ifndef CNOID_BODY_PLUGIN_BODY_SELECTION_MANAGER_H
#define CNOID_BODY_PLUGIN_BODY_SELECTION_MANAGER_H

#include <cnoid/Signal>
#include <cnoid/ItemList>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Link;
class ItemTreeWidget;

class CNOID_EXPORT BodySelectionManager
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodySelectionManager* instance();

    ~BodySelectionManager();

    SignalProxy<void(BodyItem* bodyItem, Link* link)> sigCurrentSpecified();
    SignalProxy<void(BodyItem* bodyItem)> sigCurrentBodyItemSpecified();

    SignalProxy<void(BodyItem* bodyItem, Link* link)> sigCurrentChanged();
    SignalProxy<void(BodyItem* bodyItem)> sigCurrentBodyItemChanged();

    BodyItem* currentBodyItem();
    Link* currentLink();
    void setCurrent(BodyItem* bodyItem, Link* link = nullptr, bool doSelectBodyItem = false);
    void setItemTreeWidgetToSelectCurrentBodyItem(ItemTreeWidget* itemTreeWidget);

    SignalProxy<void(const ItemList<BodyItem>& selected)> sigSelectedBodyItemsChanged();
    const ItemList<BodyItem>& selectedBodyItems() const;

    SignalProxy<void(const std::vector<bool>& selection)> sigLinkSelectionChanged(BodyItem* bodyItem);
    const std::vector<bool>& linkSelection(BodyItem* bodyItem);

    void setLinkSelection(BodyItem* bodyItem, const std::vector<bool>& linkSelection);

private:
    BodySelectionManager();
    
    class Impl;
    Impl* impl;
};

}

#endif
