#ifndef CNOID_BASE_ITEM_FILE_IO_IMPL_H
#define CNOID_BASE_ITEM_FILE_IO_IMPL_H

#include "ItemFileIO.h"
#include <QString>
#include <functional>
#include <iosfwd>

namespace cnoid {

class ItemManager;
class ItemManagerImpl;
class MessageView;

class ItemFileIO::Impl
{
public:
    ItemFileIO* self;
    int api;
    std::string formatId;
    std::vector<std::string> formatIdAliases;
    std::string caption;
    std::string fileTypeCaption;
    std::vector<std::string> extensions;
    std::function<std::string()> extensionFunction;
    ItemFileIO::InterfaceLevel interfaceLevel;
    ItemFileIO::InvocationType invocationType;
    Item* parentItem;
    Item* actuallyLoadedItem;
    std::ostream* os;
    MessageView* mv;
    std::string errorMessage;

    // This variable actualy points a instance of the ClassInfo class defined in ItemManager.cpp
    weak_ref_ptr<Referenced> classInfo;

    Impl(ItemFileIO* self, const std::string& formatId, int api);
    Impl(ItemFileIO* self, const Impl& org);

    bool isRegisteredForSingletonItem() const;
    Item* findSingletonItemInstance() const;

    bool preprocessLoadingOrSaving(
        InvocationType invocationType, Item* item, std::string& io_filename, const Mapping* options);

    bool loadItem(
        InvocationType invocationType, Item* item, std::string filename,
        Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options);

    bool saveItem(
        InvocationType invocationType, Item* item, std::string filename, const Mapping* options);

    static std::vector<std::string> separateExtensions(const std::string& multiExtString);
    static QString makeNameFilter(
        const std::string& caption, const std::vector<std::string>& extensions, bool isNayEnabled = false);
};

}

#endif
