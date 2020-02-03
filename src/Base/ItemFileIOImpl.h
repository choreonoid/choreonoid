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
    std::vector<std::string> extensions;
    std::function<std::string()> extensionFunction;
    ItemFileIO::InterfaceLevel interfaceLevel;
    ItemFileIO::InvocationType invocationType;
    Item* parentItem;
    std::ostream* os;
    MessageView* mv;

    // This variable actualy points a instance of the ClassInfo class defined in ItemManager.cpp
    weak_ref_ptr<Referenced> classInfo;

    Impl(ItemFileIO* self, const std::string& formatId, int api);
    Impl(ItemFileIO* self, const Impl& org);

    bool isFormat(const std::string& id){
        if(!id.empty()){
            if(formatId == id){
                return true;
            }
            for(auto& alias : formatIdAliases){
                if(alias == id){
                    return true;
                }
            }
        }
        return false;
    }

    std::vector<std::string> getExtensions(){
        if(extensionFunction){
            return separateExtensions(extensionFunction());
        }
        return extensions;
    }

    bool loadItem(
        InvocationType invocationType, Item* item, const std::string& filename,
        Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options);

    static std::vector<std::string> separateExtensions(const std::string& multiExtString);
    static QString makeExtensionFilter(
        const std::string& caption, const std::vector<std::string>& extensions, bool isAnyEnabled = false);
    static QStringList makeExtensionFilterList(
        const std::string& caption, const std::vector<std::string>& extensions);
    static QString makeExtensionFilterString(
        const std::string& caption, const std::vector<std::string>& extensions);
};



}

#endif
