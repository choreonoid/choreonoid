#ifndef CNOID_BASE_UNIFIED_EDIT_HISTORY_VIEW_H
#define CNOID_BASE_UNIFIED_EDIT_HISTORY_VIEW_H

#include <cnoid/View>

namespace cnoid {

class UnifiedEditHistoryView : public View
{
public:
    static void initializeClass(ExtensionManager* ext);
            
    UnifiedEditHistoryView();
    virtual ~UnifiedEditHistoryView();

    virtual void onActivated() override;
    virtual void onDeactivated() override;
            
private:
    class Impl;
    Impl* impl;
};

}

#endif
