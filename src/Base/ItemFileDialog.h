#ifndef CNOID_BASE_ITEM_FILE_DIALOG_H
#define CNOID_BASE_ITEM_FILE_DIALOG_H

#include "ItemFileIO.h"
#include "ItemList.h"
#include "FileDialog.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemFileDialog : public FileDialog
{
public:
    ItemFileDialog();
    ItemFileDialog(QWidget* parent);
    ~ItemFileDialog();

    template <class ItemType>
    void setRegisteredFileIOsFor(){
        setRegisteredFileIOsFor_(typeid(ItemType));
    }
    void setFileIOs(const std::vector<ItemFileIO*>& fileIOs);
    void setFileIO(ItemFileIO* fileIO);
    void clearFileIOs();

    ItemList<Item> loadItems(
        Item* parentItem = nullptr,
        bool doAddition = true,
        Item* nextItem = nullptr);

    void setExportMode(bool on = true);
    bool saveItem(Item* item);

    static QString makeNameFilter(
        const std::string& caption, const std::vector<std::string>& extensions, bool isAnyEnabled = false);

private:
    void setRegisteredFileIOsFor_(const std::type_info& type);

    class Impl;
    Impl* impl;
};

}

#endif
