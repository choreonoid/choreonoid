/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_LINK_SELECTION_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_SELECTION_VIEW_H

#include <cnoid/View>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;

class CNOID_EXPORT LinkSelectionView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkSelectionView* instance();
            
    LinkSelectionView();
    virtual ~LinkSelectionView();

    BodyItem* currentBodyItem();

    SignalProxy<void()> sigSelectionChanged();
    int selectedLinkIndex() const;
    const std::vector<int>& selectedLinkIndices() const;
    const std::vector<bool>& linkSelection() const;
            
    SignalProxy<void()> sigSelectionChanged(BodyItem* bodyItem);
    const std::vector<int>& selectedLinkIndices(BodyItem* bodyItem) const;
    int selectedLinkIndex(BodyItem* bodyItem) const;
    const std::vector<bool>& linkSelection(BodyItem* bodyItem) const;

#ifdef CNOID_BACKWARD_COMPATIBILITY
    const std::vector<int>& getSelectedLinkIndices(BodyItem* bodyItem) const;
    const std::vector<bool>& getLinkSelection(BodyItem* bodyItem) const;
#endif

    bool makeSingleSelection(BodyItem* bodyItem, int linkIndex);

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
    class Impl;
    Impl* impl;
};

}

#endif
