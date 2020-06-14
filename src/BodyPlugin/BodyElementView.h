#ifndef CNOID_BODY_PLUGIN_BODY_ELEMENT_VIEW_H
#define CNOID_BODY_PLUGIN_BODY_ELEMENT_VIEW_H

#include <cnoid/View>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

class CNOID_EXPORT BodyElementView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodyElementView* instance();
            
    BodyElementView();
    virtual ~BodyElementView();

    BodyItem* currentBodyItem();

    SignalProxy<void(int linkIndex)> sigCurrentLinkChanged();
    int currentLinkIndex() const;
    void setCurrentLink(int index);
    SignalProxy<void()> sigLinkSelectionChanged();
    const std::vector<int>& selectedLinkIndices() const;
    const std::vector<bool>& linkSelection() const;

    /*
    SignalProxy<void()> sigCurrentLinkChanged(BodyItem* bodyItem);
    int currentLinkIndex(BodyItem* bodyItem) const;
    void setCurrentLink(BodyItem* bodyItem, int index);
    SignalProxy<void()> sigLinkSelectionChanged(BodyItem* bodyItem);
    const std::vector<int>& selectedLinkIndices(BodyItem* bodyItem) const;
    const std::vector<bool>& linkSelection(BodyItem* bodyItem) const;
    */
    
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    class Impl;
    Impl* impl;
};

}

#endif
