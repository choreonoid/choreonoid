#include "MocapMappingItem.h"
#include <cnoid/ItemManager>
#include <cnoid/ItemFileIO>
#include <cnoid/Archive>
#include <cnoid/PutPropertyFunction>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class MocapMappingFileIO : public ItemFileIO
{
public:
    MocapMappingFileIO();
    virtual Item* createItem() override;
    virtual bool load(Item* item, const std::string& filename) override;
};

}


MocapMappingFileIO::MocapMappingFileIO()
    : ItemFileIO("MOCAP-MAPPING-YAML", Load)
{
    setCaption(_("Motion Capture Data Mapping File"));
    setFileTypeCaption(_("Motion Capture Data Mapping File (YAML)"));
    setExtension("yaml");
    addFormatAlias("CHARACTER-YAML");
}


Item* MocapMappingFileIO::createItem()
{
    return new MocapMappingItem;
}


bool MocapMappingFileIO::load(Item* item, const std::string& filename)
{
    return static_cast<MocapMappingItem*>(item)->loadMocapMapping(filename, os());
}



void MocapMappingItem::initialize(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<MocapMappingItem>(N_("MocapMappingItem"))
        .addAlias<MocapMappingItem>("CharacterItem", "Mocap")
        .addFileIO<MocapMappingItem>(new MocapMappingFileIO);
}


MocapMappingItem::MocapMappingItem()
    : mocapMapping_(new MocapMapping)
{
    setAttributes(FileImmutable | Reloadable);
}


MocapMappingItem::MocapMappingItem(const MocapMappingItem& org)
    : Item(org),
      mocapMapping_(new MocapMapping(*org.mocapMapping_))
{

}


MocapMappingItem::~MocapMappingItem()
{

}


bool MocapMappingItem::loadMocapMapping(const std::string& filename, std::ostream& os)
{
    return mocapMapping_->load(filename, os);
}


Item* MocapMappingItem::doDuplicate() const
{
    return new MocapMappingItem(*this);
}


void MocapMappingItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Number of skeleton edges"), mocapMapping_->numSkeletonEdges());
    putProperty(_("Number of skeleton segments"), mocapMapping_->numSkeletonSegments());
    putProperty(_("Number of extra markers"), mocapMapping_->numExtraMarkers());
    putProperty(_("Number of marker edges"), mocapMapping_->numMarkerEdges());
    putProperty(_("Number of marker segments"), mocapMapping_->numMarkerSegments());
}


bool MocapMappingItem::store(Archive& archive)
{
    return archive.writeFileInformation(this);
}


bool MocapMappingItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}
