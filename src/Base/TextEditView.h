/**
   @author Shizuko Hattori
*/

#ifndef CNOID_BASE_TEXT_EDIT_VIEW_H
#define CNOID_BASE_TEXT_EDIT_VIEW_H

#include <cnoid/Signal>
#include <cnoid/View>

namespace cnoid {

class TextEditViewImpl;
    
class TextEditView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    TextEditView();
    virtual ~TextEditView();

private:
    TextEditViewImpl* impl;
};

}

#endif
