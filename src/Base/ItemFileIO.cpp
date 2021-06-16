#include "ItemFileIO.h"
#include "ItemManager.h"
#include "MessageView.h"
#include <cnoid/NullOut>
#include <cnoid/ValueTree>
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;

namespace cnoid {

class ItemFileIO::Impl
{
public:
    ItemFileIO* self;
    int api;
    std::string format;
    std::vector<std::string> formatAliases;
    std::string caption;
    std::string fileTypeCaption;
    std::vector<std::string> extensions;
    std::function<std::string()> extensionFunction;
    ItemFileIO::InterfaceLevel interfaceLevel;
    int invocationType;
    Item* parentItem;
    Item* actuallyLoadedItem;
    std::ostream* os;
    MessageView* mv;
    std::string errorMessage;

    // This variable actualy points a instance of the ClassInfo class defined in ItemManager.cpp
    mutable weak_ref_ptr<Referenced> itemClassInfo;

    Impl(ItemFileIO* self, const std::string& format, int api);
    Impl(ItemFileIO* self, const Impl& org);
    bool preprocessLoadingOrSaving(Item* item, std::string& io_filename, const Mapping* options);
    bool loadItem(
        Item* item, std::string filename,
        Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options);
    bool saveItem(Item* item, std::string filename, const Mapping* options);
};

}


ItemFileIO::ItemFileIO(const std::string& format, int api)
{
    impl = new Impl(this, format, api);
}


ItemFileIO::Impl::Impl(ItemFileIO* self, const std::string& format, int api)
    : self(self),
      api(api),
      format(format)
{
    interfaceLevel = Standard;
    invocationType = Direct;
    parentItem = nullptr;
    mv = MessageView::instance();
    os = &mv->cout(true);
}


ItemFileIO::ItemFileIO(const ItemFileIO& org)
{
    impl = new Impl(this, *org.impl);
}


ItemFileIO::ItemFileIO()
{

}


void ItemFileIO::copyFrom(const ItemFileIO& org)
{
    impl = new Impl(this, *org.impl);
}


ItemFileIO::Impl::Impl(ItemFileIO* self, const Impl& org)
    : self(self),
      api(org.api),
      format(org.format),
      formatAliases(org.formatAliases),
      caption(org.caption),
      extensions(org.extensions),
      extensionFunction(org.extensionFunction),
      interfaceLevel(org.interfaceLevel),
      invocationType(org.invocationType)
{
    invocationType = Direct;
    parentItem = nullptr;
    mv = MessageView::instance();
    os = &mv->cout(true);
}
    

ItemFileIO::~ItemFileIO()
{
    delete impl;
}


bool ItemFileIO::isFormat(const std::string& format) const
{
    if(!format.empty()){
        if(impl->format == format){
            return true;
        }
        for(auto& alias : impl->formatAliases){
            if(alias == format){
                return true;
            }
        }
    }
    return false;
}


int ItemFileIO::api() const
{
    return impl->api;
}


void ItemFileIO::setApi(int api)
{
    impl->api = api;
}


bool ItemFileIO::hasApi(int api) const
{
    return impl->api & api;
}


void ItemFileIO::setCaption(const std::string& caption)
{
    impl->caption = caption;
}


const std::string& ItemFileIO::caption() const
{
    return impl->caption;
}


void ItemFileIO::setFileTypeCaption(const std::string& caption)
{
    impl->fileTypeCaption = caption;
}


const std::string& ItemFileIO::fileTypeCaption() const
{
    if(!impl->fileTypeCaption.empty()){
        return impl->fileTypeCaption;
    }
    return impl->caption;
}


void ItemFileIO::addFormatAlias(const std::string& format)
{
    impl->formatAliases.push_back(format);
}


void ItemFileIO::setExtension(const std::string& extension)
{
    impl->extensions.clear();
    impl->extensions.push_back(extension);
}


void ItemFileIO::setExtensions(const std::vector<std::string>& extensions)
{
    impl->extensions = extensions;
}


void ItemFileIO::setExtensionFunction(std::function<std::string()> func)
{
    impl->extensionFunction = func;
}


std::vector<std::string> ItemFileIO::extensions() const
{
    if(impl->extensionFunction){
        return separateExtensions(impl->extensionFunction());
    }    
    return impl->extensions;
}


std::vector<std::string> ItemFileIO::separateExtensions(const std::string& multiExtString)
{
    std::vector<std::string> extensions;
    const char* str = multiExtString.c_str();
    do {
        const char* begin = str;
        while(*str != ';' && *str) ++str;
        extensions.push_back(string(begin, str));
    } while(0 != *str++);

    return extensions;
}


void ItemFileIO::setInterfaceLevel(InterfaceLevel level)
{
    impl->interfaceLevel = level;
}


int ItemFileIO::interfaceLevel() const
{
    return impl->interfaceLevel;
}


void ItemFileIO::setInvocationType(int type)
{
    impl->invocationType = type;
}


bool ItemFileIO::Impl::preprocessLoadingOrSaving
(Item* item, std::string& io_filename, const Mapping* options)
{
    if(invocationType == Direct){
        FilePathVariableProcessor* pathProcessor = FilePathVariableProcessor::systemInstance();
        io_filename = pathProcessor->expand(io_filename, true);
        if(io_filename.empty()){
            errorMessage = pathProcessor->errorMessage();
            return false;
        }
    }

    if((invocationType == Direct) && (api & ItemFileIO::Options)){
        self->resetOptions();
        if(options){
            self->restoreOptions(options);
        }
    }

    return true;
}


Item* ItemFileIO::loadItem
(const std::string& filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    ItemPtr item = createItem();
    if(item){
        if(!loadItem(item, filename, parentItem, doAddition, nextItem, options)){
            item.reset();
        }
    }
    impl->invocationType = Direct;
    return item.retn();
}


bool ItemFileIO::loadItem
(Item* item, const std::string& filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    bool loaded = impl->loadItem(item, filename, parentItem, doAddition, nextItem, options);
    impl->invocationType = Direct;
    return loaded;
}


bool ItemFileIO::Impl::loadItem
(Item* item, std::string filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    if(filename.empty()){
        self->putError(_("Item with empty filename cannot be loaded."));
        return false;
    }

    if(!preprocessLoadingOrSaving(item, filename, options)){
        self->putError(
            fmt::format(_("{0} cannot be loaded because {1}"),
                        item->displayName(), errorMessage));
        return false;
    }
    this->parentItem = parentItem;

    mv->notify(fmt::format(_("Loading {0} \"{1}\""), caption, filename));
    mv->flush();

    actuallyLoadedItem = item;
    bool loaded = self->load(item, filename);
    mv->flush();

    if(!loaded){
        mv->put(_(" -> failed.\n"), MessageView::Highlight);
    } else {
        if(item->name().empty()){
            item->setName(toUTF8(filesystem::path(fromUTF8(filename)).stem().string()));
        }
        if(actuallyLoadedItem != item && actuallyLoadedItem->name().empty()){
            actuallyLoadedItem->setName(item->name());
        }
        MappingPtr optionArchive;
        if(api & ItemFileIO::Options){
            optionArchive = new Mapping;
            self->storeOptions(optionArchive);
        }
        actuallyLoadedItem->updateFileInformation(filename, format, optionArchive);

        if(doAddition && parentItem){
            parentItem->insertChild(nextItem, item, true);
        }
        
        mv->put(_(" -> ok!\n"));
    }
    mv->flush();

    this->parentItem = nullptr;
    actuallyLoadedItem = nullptr;

    return loaded;
}


bool ItemFileIO::load(Item* item, const std::string& filename)
{
    return false;
}


Item* ItemFileIO::createItem()
{
    return nullptr;
}


void ItemFileIO::setActuallyLoadedItem(Item* item)
{
    impl->actuallyLoadedItem = item;
}


bool ItemFileIO::saveItem(Item* item, const std::string& filename, const Mapping* options)
{
    bool saved = impl->saveItem(item, filename, options);
    impl->invocationType = Direct;
    return saved;
}


bool ItemFileIO::Impl::saveItem
(Item* item, std::string filename, const Mapping* options)
{
    if(filename.empty()){
        self->putError(
            fmt::format(_("{0} cannot be saved with empty filename."), item->displayName()));
        return false;
    }

    if(!preprocessLoadingOrSaving(item, filename, options)){
        self->putError(
            fmt::format(_("{0} cannot be saved because {1}"), item->displayName(), errorMessage));
        return false;
    }
    parentItem = item->parentItem();

    bool isExport = (interfaceLevel == Conversion);
    if(!isExport){
        mv->notify(
            fmt::format(_("Saving {0} \"{1}\" to \"{2}\""),
                        caption, item->displayName(), filename));
    } else {
        mv->notify(
            fmt::format(_("Exporting {0} \"{1}\" into \"{2}\""),
                        caption, item->displayName(), filename));
    }
    mv->flush();

    bool saved = self->save(item, filename);
    mv->flush();

    if(!saved){
        mv->put(_(" -> failed.\n"), MessageView::Highlight);

    } else {
        MappingPtr optionArchive;
        if(api & ItemFileIO::Options){
            optionArchive = new Mapping;
            self->storeOptions(optionArchive);
        }
        if(interfaceLevel != Conversion){
            item->updateFileInformation(filename, format, optionArchive);
        }
        mv->put(_(" -> ok!\n"));
    }
    mv->flush();

    this->parentItem = nullptr;

    if(saved && !isExport){
        item->setTemporal(false);
    }

    return saved;
}


bool ItemFileIO::save(Item* item, const std::string& filename)
{
    return false;
}


void ItemFileIO::resetOptions()
{

}


void ItemFileIO::storeOptions(Mapping* /* options */)
{

}


bool ItemFileIO::restoreOptions(const Mapping* /* options */)
{
    return true;
}


QWidget* ItemFileIO::getOptionPanelForLoading()
{
    return nullptr;
}


void ItemFileIO::fetchOptionPanelForLoading()
{

}


QWidget* ItemFileIO::getOptionPanelForSaving(Item* /* item */)
{
    return nullptr;
}


void ItemFileIO::fetchOptionPanelForSaving()
{

}


Item* ItemFileIO::parentItem()
{
    return impl->parentItem;
}


int ItemFileIO::invocationType() const
{
    return impl->invocationType;
}


std::ostream& ItemFileIO::os()
{
    return *impl->os;
}


void ItemFileIO::putWarning(const std::string& message)
{
    impl->mv->putln(message, MessageView::Warning);
}


void ItemFileIO::putError(const std::string& message)
{
    impl->mv->putln(message, MessageView::Error);
}


void ItemFileIO::setItemClassInfo(Referenced* info)
{
    impl->itemClassInfo = info;
}


const Referenced* ItemFileIO::itemClassInfo() const
{
    return impl->itemClassInfo.lock();
}
