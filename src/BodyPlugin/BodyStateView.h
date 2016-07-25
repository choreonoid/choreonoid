/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_STATE_VIEW_H
#define CNOID_BODY_PLUGIN_BODY_STATE_VIEW_H

#include "BodyItem.h"
#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class BodyStateViewImpl;
    
class CNOID_EXPORT BodyStateView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyStateView();
    virtual ~BodyStateView();

protected:
    virtual bool storeState(Archive& archive);
    virtual bool restoreState(const Archive& archive);

private:
    BodyStateViewImpl* impl;
};

}

#endif
