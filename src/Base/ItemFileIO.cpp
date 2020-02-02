#include "ItemFileIO.h"
#include "ItemFileIOBaseImpl.h"
#include "MessageView.h"
#include <cnoid/NullOut>

using namespace std;
using namespace cnoid;


ItemFileIOBase::ItemFileIOBase(const std::string& formatId, int api)
{
    impl = new Impl;
    impl->api = api;
    impl->formatId = formatId;
    impl->parentItem = nullptr;
    impl->mv = MessageView::instance();
    impl->os = &impl->mv->cout(true);
}


ItemFileIOBase::~ItemFileIOBase()
{
    delete impl;
}


void ItemFileIOBase::resetOptions()
{

}


void ItemFileIOBase::storeOptions(Mapping& /* archive */)
{

}


bool ItemFileIOBase::restoreOptions(Mapping& /* archive */)
{
    return true;
}


QWidget* ItemFileIOBase::optionPanelForLoading()
{
    return nullptr;
}


void ItemFileIOBase::fetchOptionPanelForLoading()
{

}


QWidget* ItemFileIOBase::optionPanelForSaving(Item* /* item */)
{
    return nullptr;
}


void ItemFileIOBase::fetchSaveOptionPanel()
{

}


void ItemFileIOBase::setCaption(const std::string& name)
{
    impl->caption = name;
}


void ItemFileIOBase::addFormatIdAlias(const std::string& formatId)
{
    impl->formatIdAlias.push_back(formatId);
}


void ItemFileIOBase::registerExtension(const std::string& extension)
{
    impl->extensions.push_back(extension);
}


void ItemFileIOBase::registerExtensions(const std::vector<std::string>& extensions)
{
    for(auto& ext : extensions){
        impl->extensions.push_back(ext);
    }
}


void ItemFileIOBase::setExtensionFunction(std::function<std::string()> func)
{
    impl->extensionFunction = func;
}

    
void ItemFileIOBase::setInterfaceLevel(InterfaceLevel level)
{
    impl->interfaceLevel = level;
}


Item* ItemFileIOBase::parentItem()
{
    return impl->parentItem;
}


ItemFileIOBase::InvocationType ItemFileIOBase::invocationType() const
{
    return impl->invocationType;
}


std::ostream& ItemFileIOBase::os()
{
    return *impl->os;
}


void ItemFileIOBase::putWarning(const std::string& message)
{
    impl->mv->put(message, MessageView::WARNING);
}


void ItemFileIOBase::putError(const std::string& message)
{
    impl->mv->put(message, MessageView::ERROR);
}
