#ifndef CNOID_BODY_PLUGIN_SIGNAL_IO_CONNECTION_MAP_ITEM_H
#define CNOID_BODY_PLUGIN_SIGNAL_IO_CONNECTION_MAP_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class SignalIoConnectionMap;

class CNOID_EXPORT SignalIoConnectionMapItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    SignalIoConnectionMapItem();
    SignalIoConnectionMapItem(const SignalIoConnectionMapItem& org);
    virtual ~SignalIoConnectionMapItem();

    SignalIoConnectionMap* connectionMap();
    const SignalIoConnectionMap* connectionMap() const;

    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<SignalIoConnectionMapItem> SignalIoConnectionMapItemPtr;

}

#endif

