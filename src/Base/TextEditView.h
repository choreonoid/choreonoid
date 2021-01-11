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

    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

protected:
    virtual void onFocusChanged(bool on) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
