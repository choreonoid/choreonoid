/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_STATE_VIEW_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_STATE_VIEW_H_INCLUDED

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
