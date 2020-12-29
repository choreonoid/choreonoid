/**
   @author Shizuko Hattori
*/

#ifndef CNOID_BASE_TEXT_EDIT_VIEW_H
#define CNOID_BASE_TEXT_EDIT_VIEW_H

#include <cnoid/View>

namespace cnoid {

class TextEditView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);

    TextEditView();
    virtual ~TextEditView();

protected:
    virtual void onFocusChanged(bool on) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
