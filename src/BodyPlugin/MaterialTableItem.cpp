#include "MaterialTableItem.h"
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Archive>
#include <cnoid/CloneMap>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include "gettext.h"

using namespace std;
using namespace cnoid;


void MaterialTableItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
        .registerClass<MaterialTableItem>(N_("MaterialTableItem"))
        .addLoader<MaterialTableItem>(
            _("Material Table"), "MATERIAL-TABLE-FILE", "yaml",
            [](MaterialTableItem* item, const std::string& filename, std::ostream& os, Item*){
                return item->materialTable_->load(filename, os);
            });
}


MaterialTableItem::MaterialTableItem()
{
    setAttributes(FileImmutable | Reloadable);
    materialTable_ = new MaterialTable;
}


MaterialTableItem::MaterialTableItem(const MaterialTableItem& org, CloneMap* cloneMap)
    : Item(org)
{
    materialTable_ = CloneMap::getClone(org.materialTable_, cloneMap);
}


Item* MaterialTableItem::doCloneItem(CloneMap* cloneMap) const
{
    return new MaterialTableItem(*this, cloneMap);
}


void MaterialTableItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty(_("Num materials"), materialTable_->numMaterials());
    putProperty(_("Num contact materials"), materialTable_->numContactMaterials());
}


bool MaterialTableItem::store(Archive& archive)
{
    return archive.writeFileInformation(this);
}


bool MaterialTableItem::restore(const Archive& archive)
{
    return archive.loadFileTo(this);
}
