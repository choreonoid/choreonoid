#include "PositionTagGroup.h"
#include "ValueTree.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class PositionTagGroup::Impl
{
public:
    std::string name;
    Signal<void(int index)> sigTagAdded;
    Signal<void(int index, PositionTag* tag)> sigTagRemoved;
    Signal<void(int index)> sigTagUpdated;
    Signal<void(const Position& T)> sigOffsetPositionChanged;
    
    Impl();
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
    impl = new Impl;

    tags_.reserve(org.tags_.size());
    for(auto& tag : org.tags_){
        append(new PositionTag(*tag));
    }
}


PositionTagGroup::~PositionTagGroup()
{
    delete impl;
}


const std::string& PositionTagGroup::name() const
{
    return impl->name;
}


void PositionTagGroup::setName(const std::string& name)
{
    impl->name = name;
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


bool PositionTagGroup::read(const Mapping* archive)
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

    clearTags();

    auto listing = archive->findListing("tags");
    if(listing->isValid()){
        for(auto& node : *listing){
            PositionTagPtr tag = new PositionTag;
            if(tag->read(node->toMapping())){
                append(tag);
            }
        }
    }
    
    return true;
}


void PositionTagGroup::write(Mapping* archive) const
{
    archive->write("type", "PositionTagGroup");
    archive->write("format_version", 1.0);

    if(!tags_.empty()){
        auto listing = archive->createListing("tags");
        for(auto& tag : tags_){
            auto node = new Mapping;
            tag->write(node);
            listing->append(node);
        }
    }
}
