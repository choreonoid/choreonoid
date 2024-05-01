#ifndef CNOID_BODY_PLUGIN_LINK_MASS_SUMMARY_VIEW_H
#define CNOID_BODY_PLUGIN_LINK_MASS_SUMMARY_VIEW_H

#include <cnoid/View>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LinkMassSummaryView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
    static LinkMassSummaryView* instance();

    LinkMassSummaryView();
    virtual ~LinkMassSummaryView();

protected:
    virtual void onActivated() override;
    virtual void onDeactivated() override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
