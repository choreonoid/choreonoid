#ifndef CNOID_BASE_ITEM_FILE_DIALOG_IO_H
#define CNOID_BASE_ITEM_FILE_DIALOG_IO_H

#include "ItemFileIO.h"
#include "ItemList.h"
#include <QDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT ItemFileDialog : public QDialog
{
public:
    ItemFileDialog();
    ~ItemFileDialog();

    ItemList<Item> loadItems(
        const std::vector<ItemFileIO*>& fileIoList,
        Item* parentItem = nullptr,
        bool doAddition = true,
        Item* nextItem = nullptr);

    void setExportMode(bool on = true);
    bool saveItem(Item* item, const std::vector<ItemFileIO*>& fileIoList);

    static QString makeNameFilter(
        const std::string& caption, const std::vector<std::string>& extensions, bool isAnyEnabled = false);

private:
    class Impl;
    Impl* impl;
};

}

#endif
