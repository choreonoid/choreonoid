#include "PositionTagGroup.h"
#include "ValueTree.h"
#include "ArchiveSession.h"
#include "EigenArchive.h"
#include "Uuid.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class PositionTagGroup::Impl
{
public:
    string name;
    Uuid uuid;
    Signal<void(int index)> sigTagAdded;
    Signal<void(int index, PositionTag* tag)> sigTagRemoved;
    Signal<void(int index)> sigTagUpdated;
    Signal<void(const Position& T)> sigOffsetPositionChanged;
    
    Impl();
    Impl(const Impl& org);
};

}


PositionTagGroup::PositionTagGroup()
    : T_offset_(Position::Identity())
{
    impl = new Impl;
}


PositionTagGroup::Impl::Impl()
{

}


PositionTagGroup::PositionTagGroup(const PositionTagGroup& org)
    : T_offset_(org.T_offset_)
{
    impl = new Impl(*org.impl);

    tags_.reserve(org.tags_.size());
    for(auto& tag : org.tags_){
        append(new PositionTag(*tag));
    }
}


PositionTagGroup::Impl::Impl(const Impl& org)
    : name(org.name)
{

}


PositionTagGroup::~PositionTagGroup()
{
    delete impl;
}


Referenced* PositionTagGroup::doClone(CloneMap*) const
{
    return new PositionTagGroup(*this);
}


const std::string& PositionTagGroup::name() const
{
    return impl->name;
}


void PositionTagGroup::setName(const std::string& name)
{
    impl->name = name;
}


const Uuid& PositionTagGroup::uuid() const
{
    return impl->uuid;
}


void PositionTagGroup::clearTags()
{
    while(!tags_.empty()){
        removeAt(tags_.size() - 1);
    }
}


void PositionTagGroup::insert(int index, PositionTag* tag)
{
    int size = tags_.size();
    if(index > size){
        index = size;
    }
    tags_.insert(tags_.begin() + index, tag);

    impl->sigTagAdded(index);
}


void PositionTagGroup::append(PositionTag* tag)
{
    insert(tags_.size(), tag);
}


bool PositionTagGroup::removeAt(int index)
{
    if(index >= tags_.size()){
        return false;
    }
    PositionTagPtr tag = tags_[index];
    tags_.erase(tags_.begin() + index);
    impl->sigTagRemoved(index, tag);
    return true;
}


SignalProxy<void(int index)> PositionTagGroup::sigTagAdded()
{
    return impl->sigTagAdded;
}


SignalProxy<void(int index, PositionTag* tag)> PositionTagGroup::sigTagRemoved()
{
    return impl->sigTagRemoved;
}


SignalProxy<void(int index)> PositionTagGroup::sigTagUpdated()
{
    return impl->sigTagUpdated;
}


SignalProxy<void(const Position& T)> PositionTagGroup::sigOffsetPositionChanged()
{
    return impl->sigOffsetPositionChanged;
}


void PositionTagGroup::notifyTagUpdate(int index)
{
    if(static_cast<size_t>(index) < tags_.size()){
        impl->sigTagUpdated(index);
    }
}


void PositionTagGroup::notifyOffsetPositionUpdate()
{
    impl->sigOffsetPositionChanged(T_offset_);
}


bool PositionTagGroup::read(const Mapping* archive, ArchiveSession* session)
{
    auto& typeNode = archive->get("type");
    if(typeNode.toString() != "PositionTagGroup"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a position tag group"), typeNode.toString()));
    }

    auto versionNode = archive->find("format_version");
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    archive->read("name", impl->name);

    impl->uuid.read(archive);
    if(!session->addReference(impl->uuid, this)){
        impl->uuid = Uuid(); // assign a new UUID to resolve the duplication
    }

    Vector3 v;
    if(cnoid::read(archive, "translation", v)){
        T_offset_.translation() = v;
    }
    if(cnoid::read(archive, "rpy", v)){
        T_offset_.linear() = rotFromRpy(radian(v));
    }

    clearTags();

    auto listing = archive->findListing("tags");
    if(listing->isValid()){
        for(auto& node : *listing){
            PositionTagPtr tag = new PositionTag;
            if(tag->read(node->toMapping(), session)){
                append(tag);
            }
        }
    }

    return true;
}


bool PositionTagGroup::write(Mapping* archive, ArchiveSession* session) const
{
    archive->write("type", "PositionTagGroup");
    archive->write("format_version", 1.0);
    archive->write("uuid", impl->uuid.toString());
    archive->write("name", impl->name);

    archive->setDoubleFormat("%.9g");
    cnoid::write(archive, "translation", T_offset_.translation());
    cnoid::write(archive, "rpy", degree(rpyFromRot(T_offset_.linear())));

    if(!tags_.empty()){
        auto listing = archive->createListing("tags");
        for(auto& tag : tags_){
            MappingPtr node = new Mapping;
            if(!tag->write(node, session)){
                return false;
            }
            listing->append(node);
        }
    }
    
    return true;
}
