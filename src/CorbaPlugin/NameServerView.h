/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_CORBAPLUGIN_NAME_SERVER_VIEW_H_INCLUDED
#define CNOID_CORBAPLUGIN_NAME_SERVER_VIEW_H_INCLUDED

#include <cnoid/View>

namespace cnoid
{
class NameServerViewImpl;
    
class NameServerView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    NameServerView();
    virtual ~NameServerView();

private:
    NameServerViewImpl* impl;
};
}

#endif
