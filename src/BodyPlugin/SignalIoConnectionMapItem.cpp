#include "SignalIoConnectionMapItem.h"
#include <cnoid/SignalIoConnectionMap>
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class SignalIoConnectionMapItem::Impl
{
public:
    SignalIoConnectionMapPtr connectionMap;

    Impl();
    Impl(const Impl& org);
};

}


void SignalIoConnectionMapItem::initializeClass(ExtensionManager* ext)
{
    auto& im = ext->itemManager();
    im.registerClass<SignalIoConnectionMapItem>(N_("SignalIoConnectionMapItem"));
    im.addCreationPanel<SignalIoConnectionMapItem>();
}


SignalIoConnectionMapItem::SignalIoConnectionMapItem()
{
    impl = new Impl;
}


SignalIoConnectionMapItem::Impl::Impl()
{
    connectionMap = new SignalIoConnectionMap;
}


SignalIoConnectionMapItem::SignalIoConnectionMapItem(const SignalIoConnectionMapItem& org)
    : Item(org)
{
    impl = new Impl(*org.impl);
}


SignalIoConnectionMapItem::Impl::Impl(const Impl& org)
{
    connectionMap = new SignalIoConnectionMap(*org.connectionMap);
}


SignalIoConnectionMapItem::~SignalIoConnectionMapItem()
{
    delete impl;
}


Item* SignalIoConnectionMapItem::doDuplicate() const
{
    return new SignalIoConnectionMapItem(*this);
}


SignalIoConnectionMap* SignalIoConnectionMapItem::connectionMap()
{
    return impl->connectionMap;
}


const SignalIoConnectionMap* SignalIoConnectionMapItem::connectionMap() const
{
    return impl->connectionMap;
}


void SignalIoConnectionMapItem::onPositionChanged()
{

}


bool SignalIoConnectionMapItem::store(Archive& archive)
{
    return true;
}


bool SignalIoConnectionMapItem::restore(const Archive& archive)
{
    return true;
}
