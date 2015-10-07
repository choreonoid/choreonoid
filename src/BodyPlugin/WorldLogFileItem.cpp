/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "WorldLogFileItem.h"
#include <cnoid/ItemManager>
#include <cnoid/Archive>
#include <cnoid/MessageView>
#include <boost/bind.hpp>
#include <fstream>
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

    fstream file;
        
    WorldLogFileItemImpl(WorldLogFileItem* self);
    WorldLogFileItemImpl(WorldLogFileItem* self, WorldLogFileItemImpl& org);
    ~WorldLogFileItemImpl();
    bool setLogFileName(const std::string& name);
    void clear();
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
    clear();
    return true;
}


void WorldLogFileItem::clear()
{
    impl->clear();
}


void WorldLogFileItemImpl::clear()
{
    if(file.is_open()){
        file.close();
    }
    file.open(filename.c_str(), ios::in | ios::out | ios::binary | ios::trunc);
}


void WorldLogFileItem::addBodyToRecord(Body* body)
{

}


int WorldLogFileItem::numBodies() const
{
    
}


const std::string& WorldLogFileItem::modelName(int bodyIndex) const
{

}


const std::string& WorldLogFileItem::bodyName(int bodyIndex) const
{

}


void WorldLogFileItem::beginFrameOutput(double time)
{

}


void WorldLogFileItem::beginBodyStatusOutput()
{

}


void WorldLogFileItem::outputLinkPositions(SE3* positions, int size)
{

}


void WorldLogFileItem::outputJointValues(double* values, int size)
{

}


void WorldLogFileItem::endFrameOutput()
{

}


bool WorldLogFileItem::readFrame(double time)
{

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
