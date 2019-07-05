/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_LINK_SELECTION_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_SELECTION_VIEW_H

#include "BodyItem.h"
#include <cnoid/View>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class LinkSelectionViewImpl;
    
class CNOID_EXPORT LinkSelectionView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkSelectionView* mainInstance();
            
    LinkSelectionView();
    virtual ~LinkSelectionView();

    BodyItem* currentBodyItem();

    SignalProxy<void()> sigSelectionChanged();
    int selectedLinkIndex() const;
    const std::vector<int>& selectedLinkIndices();
    const std::vector<bool>& linkSelection();
            
    SignalProxy<void()> sigSelectionChanged(BodyItem* bodyItem);
    const std::vector<int>& selectedLinkIndices(BodyItem* bodyItem);
    const std::vector<bool>& linkSelection(BodyItem* bodyItem);

#ifdef CNOID_BACKWARD_COMPATIBILITY
    const std::vector<int>& getSelectedLinkIndices(BodyItem* bodyItem);
    const std::vector<bool>& getLinkSelection(BodyItem* bodyItem);
#endif

    bool makeSingleSelection(BodyItem* bodyItem, int linkIndex);

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
            
    LinkSelectionViewImpl* impl;
};

}

#endif
