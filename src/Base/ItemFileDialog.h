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

private:
    class Impl;
    Impl* impl;
};

}

#endif
