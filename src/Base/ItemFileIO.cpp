#include "ItemFileIO.h"
#include "ItemFileIOImpl.h"
#include "ItemManager.h"
#include "MessageView.h"
#include <cnoid/NullOut>
#include <cnoid/FileUtil>
#include <cnoid/ValueTree>
#include <cnoid/ParametricPathProcessor>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;


ItemFileIO::ItemFileIO(const std::string& formatId, int api)
{
    impl = new Impl(this, formatId, api);
}


ItemFileIO::Impl::Impl(ItemFileIO* self, const std::string& formatId, int api)
    : self(self),
      api(api),
      formatId(formatId)
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
      formatId(org.formatId),
      formatIdAliases(org.formatIdAliases),
      caption(org.caption),
      extensions(org.extensions),
      extensionFunction(org.extensionFunction),
      interfaceLevel(org.interfaceLevel),
      invocationType(org.invocationType)
{
    parentItem = nullptr;
    mv = MessageView::instance();
    os = &mv->cout(true);
}
    

ItemFileIO::~ItemFileIO()
{
    delete impl;
}


bool ItemFileIO::isFormat(const std::string& id) const
{
    if(!id.empty()){
        if(impl->formatId == id){
            return true;
        }
        for(auto& alias : impl->formatIdAliases){
            if(alias == id){
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


bool ItemFileIO::hasApi(int api)
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


void ItemFileIO::addFormatIdAlias(const std::string& formatId)
{
    impl->formatIdAliases.push_back(formatId);
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
        return impl->separateExtensions(impl->extensionFunction());
    }    
    return impl->extensions;
}


std::vector<std::string> ItemFileIO::Impl::separateExtensions(const std::string& multiExtString)
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


QString ItemFileIO::Impl::makeNameFilter
(const std::string& caption, const std::vector<std::string>& extensions, bool isAnyEnabled)
{
    QString filter(caption.c_str());

    if(!extensions.empty()){
        QString prefix = " (";
        for(auto& ext : extensions){
            filter += prefix;
            filter += "*.";
            filter += ext.c_str();
            prefix = " ";
        }
        filter += ")";
    } else if(isAnyEnabled){
        filter += " (*)";
    }

    return filter;
}


void ItemFileIO::setInterfaceLevel(InterfaceLevel level)
{
    impl->interfaceLevel = level;
}


int ItemFileIO::interfaceLevel() const
{
    return impl->interfaceLevel;
}


bool ItemFileIO::Impl::preprocessLoadingOrSaving
(InvocationType invocationType, Item* item, std::string& io_filename, const Mapping* options)
{
    ParametricPathProcessor* pathProcessor = ParametricPathProcessor::instance();
    auto expanded = pathProcessor->expand(io_filename);
    if(!expanded){
        errorMessage = pathProcessor->errorMessage();
        return false;
    }

    filesystem::path filepath = cnoid::getAbsolutePath(*expanded);
    io_filename = toActualPathName(cnoid::getPathString(filepath));

    this->invocationType = invocationType;
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
    if(auto item = createItem()){
        if(loadItem(item, filename, parentItem, doAddition, nextItem, options)){
            return item;
        }
    }
    return nullptr;
}


bool ItemFileIO::loadItem
(Item* item, const std::string& filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    return impl->loadItem(Direct, item, filename, parentItem, doAddition, nextItem, options);
}


bool ItemFileIO::Impl::loadItem
(InvocationType invocationType, Item* item, std::string filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    if(filename.empty()){
        self->putError(_("Item with empty filename cannot be loaded."));
        return false;
    }

    if(!preprocessLoadingOrSaving(invocationType, item, filename, options)){
        self->putError(format(_("{0} cannot be loaded because {1}"), item->name(), errorMessage));
        return false;
    }
    this->parentItem = parentItem;

    mv->notify(format(_("Loading {0} \"{1}\""), caption, filename));
    mv->flush();

    actuallyLoadedItem = item;
    bool loaded = self->load(item, filename);
    mv->flush();

    if(!loaded){
        mv->put(_(" -> failed.\n"), MessageView::HIGHLIGHT);
    } else {
        if(item->name().empty()){
            item->setName(filesystem::path(filename).stem().string());
        }
        if(actuallyLoadedItem != item && actuallyLoadedItem->name().empty()){
            actuallyLoadedItem->setName(item->name());
        }
        MappingPtr optionArchive;
        if(api & ItemFileIO::Options){
            optionArchive = new Mapping;
            self->storeOptions(optionArchive);
        }
        actuallyLoadedItem->updateFileInformation(filename, formatId, optionArchive);

        if(doAddition && parentItem){
            parentItem->insertChildItem(item, nextItem, true);
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


void ItemFileIO::setActuallyLoadedItem(Item* item)
{
    impl->actuallyLoadedItem = item;
}


bool ItemFileIO::saveItem(Item* item, const std::string& filename, const Mapping* options)
{
    return impl->saveItem(Direct, item, filename, options);
}


bool ItemFileIO::Impl::saveItem
(InvocationType invocationType, Item* item, std::string filename, const Mapping* options)
{
    if(filename.empty()){
        self->putError(format(_("{0} cannot be saved with empty filename."), item->name()));
        return false;
    }

    if(!preprocessLoadingOrSaving(invocationType, item, filename, options)){
        self->putError(format(_("{0} cannot be saved because {1}"), item->name(), errorMessage));
        return false;
    }
    parentItem = item->parentItem();

    bool isExport = (interfaceLevel == Conversion);
    if(!isExport){
        mv->notify(format(_("Saving {0} \"{1}\" to \"{2}\""),
                          caption, item->name(), filename));
    } else {
        mv->notify(format(_("Exporting {0} \"{1}\" into \"{2}\""),
                          caption, item->name(), filename));
    }
    mv->flush();

    bool saved = self->save(item, filename);
    mv->flush();

    if(!saved){
        mv->put(_(" -> failed.\n"), MessageView::HIGHLIGHT);

    } else {
        MappingPtr optionArchive;
        if(api & ItemFileIO::Options){
            optionArchive = new Mapping;
            self->storeOptions(optionArchive);
        }
        if(interfaceLevel != Conversion){
            item->updateFileInformation(filename, formatId, optionArchive);
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


ItemFileIO::InvocationType ItemFileIO::invocationType() const
{
    return impl->invocationType;
}


bool ItemFileIO::isRegisteredForSingletonItem() const
{
    return impl->isRegisteredForSingletonItem();
}


Item* ItemFileIO::findSingletonItemInstance() const
{
    return impl->findSingletonItemInstance();
}


std::ostream& ItemFileIO::os()
{
    return *impl->os;
}


void ItemFileIO::putWarning(const std::string& message)
{
    impl->mv->putln(message, MessageView::WARNING);
}


void ItemFileIO::putError(const std::string& message)
{
    impl->mv->putln(message, MessageView::ERROR);
}


ItemFileIOExtenderBase::ItemFileIOExtenderBase(const std::type_info& type, const std::string& formatId)
{
    baseFileIO = ItemManager::findFileIO(type, formatId);
    if(baseFileIO){
        copyFrom(*baseFileIO);
    }
}


bool ItemFileIOExtenderBase::isAvailable() const
{
    return baseFileIO != nullptr;
}


bool ItemFileIOExtenderBase::load(Item* item, const std::string& filename)
{
    return baseFileIO ? baseFileIO->load(item, filename) : false;
}


void ItemFileIOExtenderBase::resetOptions()
{
    if(baseFileIO){
        baseFileIO->resetOptions();
    }
}


void ItemFileIOExtenderBase::storeOptions(Mapping* options)
{
    if(baseFileIO){
        baseFileIO->storeOptions(options);
    }
}


bool ItemFileIOExtenderBase::restoreOptions(const Mapping* options)
{
    return baseFileIO ? baseFileIO->restoreOptions(options) : true;
}


QWidget* ItemFileIOExtenderBase::getOptionPanelForLoading()
{
    return baseFileIO ? baseFileIO->getOptionPanelForLoading() : nullptr;
}


void ItemFileIOExtenderBase::fetchOptionPanelForLoading()
{
    if(baseFileIO){
        baseFileIO->fetchOptionPanelForLoading();
    }
}


QWidget* ItemFileIOExtenderBase::getOptionPanelForSaving(Item* item)
{
    return baseFileIO ? baseFileIO->getOptionPanelForSaving(item) : nullptr;
}


void ItemFileIOExtenderBase::fetchOptionPanelForSaving()
{
    if(baseFileIO){
        baseFileIO->fetchOptionPanelForSaving();
    }
}
    

bool ItemFileIOExtenderBase::save(Item* item, const std::string& filename)
{
    return baseFileIO ? baseFileIO->save(item, filename) : false;
}
