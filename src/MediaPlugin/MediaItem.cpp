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
#include <cnoid/FileUtil>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/filesystem.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace boost::lambda;
using namespace cnoid;

namespace {

bool loadMediaItem(MediaItemPtr item, const std::string& filepath, std::ostream& os, Item* parentItem)
{
    bool loaded = item->setFilepath(filepath);
    if(!loaded){
        os << item->lastErrorMessage() << endl;
    }
    return loaded;
}

void onSigOptionsParsed(program_options::variables_map& v)
{
    if(v.count("media")){
        vector<string> mediaFilenames = v["media"].as< vector<string> >();
            
        for(size_t i=0; i < mediaFilenames.size(); ++i){
            MediaItemPtr item(new MediaItem());
            if(item->setFilepath(mediaFilenames[i])){
                RootItem::mainInstance()->addChildItem(item);
            }
        }
    }
}
}


void MediaItem::initialize(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MediaItem>(N_("MediaItem"));
    ext->itemManager().addLoader<MediaItem>(_("Media file"), "MEDIA-GENERIC", "", loadMediaItem);

    ext->optionManager().addOption("media", program_options::value< vector<string> >(), _("load an media file"));
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

    if(!org.filepath().empty()){
        setFilepath(org.filepath());
    } else {
        setUri(org.uri());
    }
}


MediaItem::~MediaItem()
{

}


bool MediaItem::setUri(const std::string& uri)
{
    uri_ = uri;
    filepath_.clear(); /// \todo set filenae_ when uri points a local file path
    return true;
}
    

bool MediaItem::setFilepath(const std::string& filepath)
{
    filepath_.clear();
    uri_.clear();
    
    filesystem::path fpath(filepath);

    if(filesystem::exists(fpath) && !filesystem::is_directory(fpath)){
        filepath_ = filepath;
        filesystem::path fullpath = getAbsolutePath(fpath);
        uri_ = str(fmt("file://%1%") % getPathString(fullpath));
        return true;

    } else {
        lastErrorMessage_ = str(fmt(_("Media file \"%1%\" does not exist.")) % filepath);
        return false;
    }
}


void MediaItem::setOffsetTime(double offset)
{
    offsetTime_ = offset;
}


ItemPtr MediaItem::doDuplicate() const
{
    return new MediaItem(*this);
}


void MediaItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("uri"), uri_);
    putProperty(_("offset"), offsetTime_, (bind(&MediaItem::setOffsetTime, this, _1), true));
}


bool MediaItem::store(Archive& archive)
{
    if(!filepath_.empty()){
        archive.writeRelocatablePath("file", filepath_);
        if(!lastAccessedFileFormatId().empty()){
            archive.write("format", lastAccessedFileFormatId());
        }
    } else if(!uri_.empty()){
        archive.write("uri", uri_, DOUBLE_QUOTED);
    }
    archive.write("offsetTime", offsetTime_);

    return true;
}


bool MediaItem::restore(const Archive& archive)
{
    bool restored = false;
    
    string location;
    string format = "MEDIA-GENERIC";
    
    if(archive.readRelocatablePath("file", location)){
        archive.read("format", format);
        restored = load(location, format);
    } else {
        restored = setUri(archive.get("uri", ""));
    }

    setOffsetTime(archive.get("offsetTime", 0.0));

    return restored;
}
