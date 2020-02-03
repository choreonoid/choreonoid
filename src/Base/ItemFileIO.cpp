#include "ItemFileIO.h"
#include "ItemFileIOImpl.h"
#include "ItemManager.h"
#include "RootItem.h"
#include "MainWindow.h"
#include "MessageView.h"
#include "AppConfig.h"
#include <cnoid/NullOut>
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <cnoid/ParametricPathProcessor>
#include <QFileDialog>
#include <QBoxLayout>
#include <QStyle>
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


int ItemFileIO::api() const
{
    return impl->api;
}


void ItemFileIO::setApi(int api)
{
    impl->api = api;
}

void ItemFileIO::setCaption(const std::string& name)
{
    impl->caption = name;
}


void ItemFileIO::addFormatIdAlias(const std::string& formatId)
{
    impl->formatIdAliases.push_back(formatId);
}


void ItemFileIO::setExtension(const std::string& extension)
{
    impl->extensions.push_back(extension);
}


void ItemFileIO::setExtensions(const std::vector<std::string>& extensions)
{
    for(auto& ext : extensions){
        impl->extensions.push_back(ext);
    }
}


void ItemFileIO::setExtensionFunction(std::function<std::string()> func)
{
    impl->extensionFunction = func;
}

    
void ItemFileIO::setInterfaceLevel(InterfaceLevel level)
{
    impl->interfaceLevel = level;
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


ItemList<Item> ItemFileIO::loadItemsWithDialog
(Item* parentItem, bool doAddition, Item* nextItem)
{
    ItemList<Item> loadedItems;

    bool isSingleton = false;
    ItemPtr item = ItemManager::singletonInstance(this);
    if(item){
        isSingleton = true;
        if(item->parentItem()){
            showWarningDialog(
                format(_("The singleton instance of {} has already been loaded."),
                       impl->caption));
            return loadedItems;
        }
    }

    QDialog dialog(MainWindow::instance());
    dialog.setWindowTitle(QString(_("Load %1")).arg(impl->caption.c_str()));
    dialog.setSizeGripEnabled(true);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    dialog.setLayout(vbox);
    
    QFileDialog fileDialog(&dialog);
    fileDialog.setWindowFlags(fileDialog.windowFlags() & ~Qt::Dialog);
    fileDialog.setOption(QFileDialog::DontUseNativeDialog);
    fileDialog.setSizeGripEnabled(false);
    fileDialog.setViewMode(QFileDialog::List);
    fileDialog.setLabelText(QFileDialog::Accept, _("Open"));
    fileDialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    fileDialog.setDirectory(AppConfig::archive()->get
                        ("currentFileDialogDirectory", shareDirectory()).c_str());

    QObject::connect(&fileDialog, SIGNAL(finished(int)), &dialog, SLOT(done(int)));

    vbox->addWidget(&fileDialog);

    QWidget* optionPanel = nullptr;
    if(impl->api & ItemFileIO::Options){
        resetOptions();
        if(impl->api & ItemFileIO::OptionPanelForLoading){
            optionPanel = getOptionPanelForLoading();
            if(optionPanel){
                auto hbox = new QHBoxLayout;
                auto style = dialog.style();
                int left = style->pixelMetric(QStyle::PM_LayoutLeftMargin);
                int right = style->pixelMetric(QStyle::PM_LayoutRightMargin);
                int bottom = style->pixelMetric(QStyle::PM_LayoutBottomMargin);
                hbox->setContentsMargins(left, 0, right, bottom);
                hbox->addWidget(optionPanel);
                hbox->addStretch();
                vbox->addLayout(hbox);
            }
        }
    }

    QStringList filters;
    if(!impl->extensions.empty()){
        filters = impl->makeExtensionFilterList(impl->caption, impl->extensions);
    } else if(impl->extensionFunction){
        filters = impl->makeExtensionFilterList(
            impl->caption, impl->separateExtensions(impl->extensionFunction()));
    }
    fileDialog.setNameFilters(filters);

    if(isSingleton){
        fileDialog.setFileMode(QFileDialog::ExistingFile);
    } else {
        fileDialog.setFileMode(QFileDialog::ExistingFiles);
    }

    if(dialog.exec() == QDialog::Accepted){
        Mapping* config = AppConfig::archive();
        config->writePath(
            "currentFileDialogDirectory",
            fileDialog.directory().absolutePath().toStdString());
                  
        QStringList filenames = fileDialog.selectedFiles();

        if(!parentItem){
            parentItem = RootItem::instance();
        }

        if(optionPanel){
            fetchOptionPanelForLoading();
        }
        
        for(int i=0; i < filenames.size(); ++i){
            if(!isSingleton){
                item = createItem();
            }
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));
            if(impl->loadItem(Dialog, item, filename, parentItem, doAddition, nextItem, nullptr)){
                loadedItems.push_back(item);
            }
        }
    }

    if(optionPanel){
        optionPanel->setParent(nullptr);
    }

    return loadedItems;
}
    

bool ItemFileIO::Impl::loadItem
(InvocationType invocationType, Item* item, const std::string& filename,
 Item* parentItem, bool doAddition, Item* nextItem, const Mapping* options)
{
    if(filename.empty()){
        self->putError(_("Item with empty filename cannot be loaded."));
        return false;
    }

    ParametricPathProcessor* pathProcessor = ParametricPathProcessor::instance();
    auto expanded = pathProcessor->expand(filename);
    if(!expanded){
        self->putError(pathProcessor->errorMessage());
        return false;
    }

    filesystem::path filepath = cnoid::getAbsolutePath(*expanded);
    string pathString = cnoid::getPathString(filepath);

    this->invocationType = invocationType;
    if((invocationType == Direct) && (api & ItemFileIO::Options)){
        self->resetOptions();
        if(options){
            self->restoreOptions(options);
        }
    }

    string actualFilename(toActualPathName(pathString));

    mv->notify(format(_("Loading {0} \"{1}\""), caption, actualFilename));
    mv->flush();

    if(parentItem){
        this->parentItem = parentItem;
    } else {
        this->parentItem = RootItem::instance();
    }

    bool loaded = self->load(item, actualFilename);
    os->flush();

    if(!loaded){
        mv->put(_(" -> failed.\n"), MessageView::HIGHLIGHT);
    } else {
        if(item->name().empty()){
            item->setName(filesystem::path(filename).stem().string());
        }
        MappingPtr optionArchive;
        if(api & ItemFileIO::Options){
            optionArchive = new Mapping;
            self->storeOptions(optionArchive);
        }
        item->updateFileInformation(actualFilename, formatId, optionArchive);

        if(doAddition){
            parentItem->insertChildItem(item, nextItem, true);
        }
        
        mv->put(_(" -> ok!\n"));
    }
    mv->flush();

    return loaded;
}


/*
bool ItemFileIO::saveItem(Item* item, const std::string& filename)
{

}
*/


bool ItemFileIO::load(Item* item, const std::string& filename)
{
    return false;
}


void ItemFileIO::resetOptions()
{

}


void ItemFileIO::storeOptions(Mapping* /* archive */)
{

}


bool ItemFileIO::restoreOptions(const Mapping* /* archive */)
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


bool ItemFileIO::save(Item* item, const std::string& filename)
{
    return false;
}


QWidget* ItemFileIO::getOptionPanelForSaving(Item* /* item */)
{
    return nullptr;
}


void ItemFileIO::fetchSaveOptionPanel()
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

void ItemFileIOExtenderBase::storeOptions(Mapping* archive)
{
    if(baseFileIO){
        baseFileIO->storeOptions(archive);
    }
}

bool ItemFileIOExtenderBase::restoreOptions(const Mapping* archive)
{
    return baseFileIO ? baseFileIO->restoreOptions(archive) : true;
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

void ItemFileIOExtenderBase::fetchSaveOptionPanel()
{
    if(baseFileIO){
        baseFileIO->fetchSaveOptionPanel();
    }
}
    

bool ItemFileIOExtenderBase::save(Item* item, const std::string& filename)
{
    return baseFileIO ? baseFileIO->save(item, filename) : false;
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


QString ItemFileIO::Impl::makeExtensionFilter
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


QStringList ItemFileIO::Impl::makeExtensionFilterList
(const std::string& caption, const std::vector<std::string>& extensions)
{
    QStringList filters;
    QString filter = makeExtensionFilter(caption, extensions);
    if(!filter.isEmpty()){
        filters << filter;
    }
    filters << _("Any files (*)");
    return filters;
}


QString ItemFileIO::Impl::makeExtensionFilterString
(const std::string& caption, const std::vector<std::string>& extensions)
{
    QString filters = makeExtensionFilter(caption, extensions);
    filters += _(";;Any files (*)");
    return filters;
}
