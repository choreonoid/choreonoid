/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_BODY_IO_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_BODY_IO_RTC_ITEM_H

#include "ControllerRTCItem.h"
#include "exportdecl.h"

namespace cnoid {

class BodyIoRTCItemImpl;

class CNOID_EXPORT BodyIoRTCItem : public ControllerRTCItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyIoRTCItem();
    BodyIoRTCItem(const BodyIoRTCItem& org);
    virtual ~BodyIoRTCItem();

    virtual bool initialize(ControllerIO* io) override;
    virtual bool start() override;
    virtual void input() override;
    virtual void output() override;
    virtual void stop() override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onConnectedToRoot() override;
    virtual void onPositionChanged() override;
    virtual void onOptionsChanged() override;
    virtual std::string getDefaultRTCInstanceName() const override;
    virtual bool createRTC() override;
    virtual void deleteRTC(bool waitToBeDeleted) override;

private:
    BodyIoRTCItemImpl* impl;
    friend class BodyIoRTCItemImpl;
};

typedef ref_ptr<BodyIoRTCItem> BodyIoRTCItemPtr;

}
#endif
