/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldLogFileItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

}


namespace cnoid {

class WorldLogFileItemImpl
{
public:
    WorldLogFileItem* self;
    string filename;
        
    WorldLogFileItemImpl(WorldLogFileItem* self);
    WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org);
    ~WorldLogFileItemImpl();
    bool setLogFileName(const std::string& name);
};

}


void WorldLogFileItem::initializeClass(ExtensionManager* ext)
{
    ItemManager& im = ext->itemManager();
    im.registerClass<WorldLogFileItem>(N_("WorldLogFileItem"));
    im.addCreationPanel<WorldLogFileItem>();
}


WorldLogFileItem::WorldLogFileItem()
{
    impl = new WorldLogFileItemImpl(this);
}


WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self)
    : self(self)
{

}


WorldLogFileItem::WorldLogFileItem(const WorldLogFileItem& org)
    : Item(org)
{
    impl = new WorldLogFileItemImpl(this, *org.impl);
}


WorldLogFileItemImpl::WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org)
    : self(self)
{
    filename = org.filename;
}


WorldLogFileItem::~WorldLogFileItem()
{
    delete impl;
}


WorldLogFileItemImpl::~WorldLogFileItemImpl()
{

}


ItemPtr WorldLogFileItem::doDuplicate() const
{
    return new WorldLogFileItem(*this);
}


void WorldLogFileItem::notifyUpdate()
{
    Item::notifyUpdate();
}


const std::string& WorldLogFileItem::logFileName() const
{
    return impl->filename;
}


bool WorldLogFileItem::setLogFileName(const std::string& filename)
{
    return impl->setLogFileName(filename);
}


bool WorldLogFileItemImpl::setLogFileName(const std::string& name)
{
    filename = name;
    return true;
}


void WorldLogFileItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Log file"), impl->filename,
                boost::bind(&WorldLogFileItemImpl::setLogFileName, impl, _1));
}


bool WorldLogFileItem::store(Archive& archive)
{
    archive.writeRelocatablePath("filename", impl->filename);
    return true;
}


bool WorldLogFileItem::restore(const Archive& archive)
{
    string filename;
    if(archive.readRelocatablePath("filename", filename)){
        impl->setLogFileName(filename);
    }
    return true;
}
