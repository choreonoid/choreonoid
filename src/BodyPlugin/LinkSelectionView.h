/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_LINK_SELECTION_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_LINK_SELECTION_VIEW_H_INCLUDED

#include "BodyItem.h"
#include <cnoid/View>
#include <boost/dynamic_bitset.hpp>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class LinkSelectionViewImpl;
    
class CNOID_EXPORT LinkSelectionView : public cnoid::View, public boost::signals::trackable
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkSelectionView* mainInstance();
            
    LinkSelectionView();
    virtual ~LinkSelectionView();
            
    SignalProxy< boost::signal<void()> > sigSelectionChanged(BodyItemPtr bodyItem);

    const std::vector<int>& getSelectedLinkIndices(BodyItemPtr bodyItem);
    const boost::dynamic_bitset<>& getLinkSelection(BodyItemPtr bodyItem);

    bool makeSingleSelection(BodyItemPtr bodyItem, int linkIndex);

    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);
            
private:
            
    LinkSelectionViewImpl* impl;
};
}

#endif
