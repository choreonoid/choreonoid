/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_STATE_VIEW_H
#define CNOID_BODY_PLUGIN_BODY_STATE_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

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
    class Impl;
    Impl* impl;
};

}

#endif
