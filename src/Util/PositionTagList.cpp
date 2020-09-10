#include "PositionTagList.h"
#include "ValueTree.h"
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class PositionTagList::Impl
{
public:
    Signal<void(int index)> sigTagAdded;
    Signal<void(int index, PositionTag* tag)> sigTagRemoved;
    Signal<void(int index)> sigTagUpdated;

    Impl();
};

}


PositionTagList::PositionTagList()
{
    impl = new Impl;
}


PositionTagList::Impl::Impl()
{

}


PositionTagList::PositionTagList(const PositionTagList& org)
{
    impl = new Impl;

    tags_.reserve(org.tags_.size());
    for(auto& tag : org.tags_){
        append(new PositionTag(*tag));
    }
}


PositionTagList::~PositionTagList()
{
    delete impl;
}


void PositionTagList::clear()
{
    while(!tags_.empty()){
        removeAt(tags_.size() - 1);
    }
}


void PositionTagList::insert(int index, PositionTag* tag)
{
    int size = tags_.size();
    if(index > size){
        index = size;
    }
    tags_.insert(tags_.begin() + index, tag);

    impl->sigTagAdded(index);
}


void PositionTagList::append(PositionTag* tag)
{
    insert(tags_.size(), tag);
}


bool PositionTagList::removeAt(int index)
{
    if(index >= tags_.size()){
        return false;
    }
    PositionTagPtr tag = tags_[index];
    tags_.erase(tags_.begin() + index);
    impl->sigTagRemoved(index, tag);
    return true;
}


SignalProxy<void(int index)> PositionTagList::sigTagAdded()
{
    return impl->sigTagAdded;
}


SignalProxy<void(int index, PositionTag* tag)> PositionTagList::sigTagRemoved()
{
    return impl->sigTagRemoved;
}


SignalProxy<void(int index)> PositionTagList::sigTagUpdated()
{
    return impl->sigTagUpdated;
}


bool PositionTagList::read(const Mapping* archive)
{
    auto& typeNode = archive->get("type");
    if(typeNode.toString() != "PositionTagList"){
        typeNode.throwException(
            format(_("{0} cannot be loaded as a position tag list"), typeNode.toString()));
    }

    auto versionNode = archive->find("format_version");
    auto version = versionNode->toDouble();
    if(version != 1.0){
        versionNode->throwException(format(_("Format version {0} is not supported."), version));
    }

    clear();

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


void PositionTagList::write(Mapping* archive) const
{
    archive->write("type", "PositionTagList");
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
