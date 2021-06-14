/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MediaItem.h"
#include <cnoid/ItemManager>
#include <cnoid/OptionManager>
#include <cnoid/RootItem>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace std::placeholders;
using namespace cnoid;
using fmt::format;
namespace filesystem = cnoid::stdx::filesystem;

namespace {

bool loadMediaItem(MediaItemPtr item, const std::string& filepath, std::ostream& os, Item* parentItem)
{
    bool loaded = item->setMediaFilePath(filepath);
    if(!loaded){
        os << item->lastErrorMessage() << endl;
    }
    return loaded;
}

void onSigOptionsParsed(boost::program_options::variables_map& v)
{
    if(v.count("media")){
        vector<string> mediaFilenames = v["media"].as< vector<string> >();
            
        for(size_t i=0; i < mediaFilenames.size(); ++i){
            MediaItemPtr item(new MediaItem());
            if(item->setMediaFilePath(mediaFilenames[i])){
                RootItem::instance()->addChildItem(item);
            }
        }
    }
}

}


void MediaItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MediaItem>(N_("MediaItem"));
    ext->itemManager().addLoader<MediaItem>(_("Media file"), "MEDIA-GENERIC", "", loadMediaItem);

    ext->optionManager().addOption("media", boost::program_options::value< vector<string> >(), _("load an media file"));
    ext->optionManager().sigOptionsParsed().connect(onSigOptionsParsed);
}


MediaItem::MediaItem()
{
    offsetTime_ = 0.0;
}


MediaItem::MediaItem(const MediaItem& org)
    : Item(org)
{
    offsetTime_ = org.offsetTime_;

    if(!org.mediaFilePath().empty()){
        setMediaFilePath(org.mediaFilePath());
    } else {
        setMediaURI(org.mediaURI());
    }
}


MediaItem::~MediaItem()
{

}


bool MediaItem::setMediaURI(const std::string& uri)
{
    mediaURI_ = uri;
    mediaFilePath_.clear(); /// \todo set filenae_ when uri points a local file path
    return true;
}
    

bool MediaItem::setMediaFilePath(const std::string& filepath)
{
    mediaFilePath_.clear();
    mediaURI_.clear();
    
    filesystem::path fpath(fromUTF8(filepath));

    if(filesystem::exists(fpath) && !filesystem::is_directory(fpath)){
        mediaFilePath_ = filepath;
        filesystem::path fullpath(filesystem::absolute(fpath));
        mediaURI_ = format("file://{}", toUTF8(fullpath.generic_string()));
        return true;

    } else {
        lastErrorMessage_ = format(_("Media file \"{}\" does not exist."), filepath);
        return false;
    }
}


void MediaItem::setOffsetTime(double offset)
{
    offsetTime_ = offset;
}


Item* MediaItem::doDuplicate() const
{
    return new MediaItem(*this);
}


void MediaItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty("uri", mediaURI_);
    putProperty(_("offset"), offsetTime_, [&](double value){ setOffsetTime(value); return true; });
}


bool MediaItem::store(Archive& archive)
{
    if(!mediaFilePath_.empty()){
        archive.writeRelocatablePath("file", mediaFilePath_);
        if(!fileFormat().empty()){
            archive.write("format", fileFormat());
        }
    } else if(!mediaURI_.empty()){
        archive.write("uri", mediaURI_, DOUBLE_QUOTED);
    }
    archive.write("offset_time", offsetTime_);
    return true;
}


bool MediaItem::restore(const Archive& archive)
{
    bool restored = archive.loadFileTo(this);
    if(!restored){
        restored = setMediaURI(archive.get("uri", ""));
    }
    setOffsetTime(archive.get({ "offset_time", "offsetTime" }, 0.0));
    return restored;
}
