#ifndef CNOID_BODY_PLUGIN_BODY_LINK_VIEW_H
#define CNOID_BODY_PLUGIN_BODY_LINK_VIEW_H

#include <cnoid/View>

namespace cnoid {

class BodyLinkView : public cnoid::View
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyLinkView();
    virtual ~BodyLinkView();

    void setQuaternionMode(bool on);

protected:
    virtual void onAttachedMenuRequest(MenuManager& menu) override;
    virtual bool storeState(Archive& archive) override;
    virtual bool restoreState(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
