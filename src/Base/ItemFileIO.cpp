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
        this->parentItem = nullptr;
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

        if(doAddition && parentItem){
            parentItem->insertChildItem(item, nextItem, true);
        }
        
        mv->put(_(" -> ok!\n"));
    }
    mv->flush();

    this->parentItem = nullptr;

    return loaded;
}


namespace cnoid {

class ItemFileIO::Dialog::Impl
{
    Dialog* self;
    QFileDialog fileDialog;
    const vector<ItemFileIO*>* pFileIoList;
    ItemFileIO* targetFileIO;
    QWidget* optionPanel;
    QBoxLayout* optionPanelBox;
    bool isSingletonItem;
    
public:
    Impl(Dialog* self);
    ItemList<Item>  loadItems(
        const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem);

private:
    void setTargetFileIO(ItemFileIO* fileIO);
    void onFilterSelected(const QString& filter);
};

}

ItemFileIO::Dialog::Dialog()
    : QDialog(MainWindow::instance())
{
    impl = new Impl(this);
}


ItemFileIO::Dialog::~Dialog()
{
    delete impl;
}


ItemFileIO::Dialog::Impl::Impl(Dialog* self)
    : self(self),
      fileDialog(self)
{
    pFileIoList = nullptr;
    targetFileIO = nullptr;
    
    self->setSizeGripEnabled(true);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    self->setLayout(vbox);

    fileDialog.setWindowFlags(fileDialog.windowFlags() & ~Qt::Dialog);
    fileDialog.setOption(QFileDialog::DontUseNativeDialog);
    fileDialog.setSizeGripEnabled(false);
    fileDialog.setViewMode(QFileDialog::List);
    fileDialog.setLabelText(QFileDialog::Accept, _("Open"));
    fileDialog.setLabelText(QFileDialog::Reject, _("Cancel"));
    fileDialog.setDirectory(AppConfig::archive()->get
                        ("file_dialog_directory", shareDirectory()).c_str());
    vbox->addWidget(&fileDialog);

    optionPanel = nullptr;
    optionPanelBox = new QHBoxLayout;
    auto sty = self->style();
    int left = sty->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int right = sty->pixelMetric(QStyle::PM_LayoutRightMargin);
    int bottom = sty->pixelMetric(QStyle::PM_LayoutBottomMargin);
    optionPanelBox->setContentsMargins(left, 0, right, bottom);
    optionPanelBox->addStretch();
    vbox->addLayout(optionPanelBox);

    QObject::connect(&fileDialog, &QFileDialog::filterSelected,
                     [&](const QString& filter){ onFilterSelected(filter); });
    
    QObject::connect(&fileDialog, SIGNAL(finished(int)), self, SLOT(done(int)));
}


ItemList<Item> ItemFileIO::Dialog::loadItems
(const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem)
{
    return impl->loadItems(fileIoList, parentItem, doAddition, nextItem);
}


ItemList<Item> ItemFileIO::Dialog::Impl::loadItems
(const vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem)
{
    ItemList<Item> loadedItems;

    if(fileIoList.empty()){
        return loadedItems;
    }
    pFileIoList = &fileIoList;

    QStringList filters;
    for(auto& fileIO : fileIoList){
        filters << ItemFileIO::Impl::makeNameFilter(fileIO->fileTypeCaption(), fileIO->extensions());
    }
    if(filters.size() == 1){
        filters << _("Any files (*)");
    } else {
        // add "any file" filters for the file ios that supports it
    }
    fileDialog.setNameFilters(filters);

    if(self->windowTitle().isEmpty()){
        self->setWindowTitle(QString(_("Load %1")).arg(fileIoList.front()->caption().c_str()));
    }
    setTargetFileIO(fileIoList.front());

    if(self->exec() == QDialog::Accepted){

        AppConfig::archive()->writePath(
            "file_dialog_directory",
            fileDialog.directory().absolutePath().toStdString());

        QStringList filenames = fileDialog.selectedFiles();

        if(!parentItem){
            parentItem = RootItem::instance();
        }

        if(optionPanel){
            targetFileIO->fetchOptionPanelForLoading();
        }

        bool isSingleton = false;
        ItemPtr item = targetFileIO->findSingletonItemInstance();
        if(item){
            isSingleton = true;
            if(item->parentItem()){
                showWarningDialog(
                    format(_("The singleton instance of {} has already been loaded."),
                           targetFileIO->caption()));
                goto exit;
            }
        }

        for(int i=0; i < filenames.size(); ++i){
            if(!isSingleton){
                item = targetFileIO->createItem();
            }
            string filename = getNativePathString(filesystem::path(filenames[i].toStdString()));
            bool loaded = targetFileIO->impl->loadItem(
                ItemFileIO::Dialog, item, filename, parentItem, doAddition, nextItem, nullptr);
            if(loaded){
                loadedItems.push_back(item);
            }
        }
    }

exit:
    for(auto& fileIO : fileIoList){
        if(auto panel = fileIO->getOptionPanelForLoading()){
            panel->setParent(nullptr);
        }
    }

    return loadedItems;
}


void ItemFileIO::Dialog::Impl::setTargetFileIO(ItemFileIO* fileIO)
{
    targetFileIO = fileIO;

    //self->setWindowTitle(QString(_("Load %1")).arg(fileIO->caption().c_str()));

    if(optionPanel){
        optionPanel->setParent(nullptr);
        optionPanel = nullptr;
    }
    int api = fileIO->api();
    if(api & ItemFileIO::Options){
        fileIO->resetOptions();
        if(api & ItemFileIO::OptionPanelForLoading){
            optionPanel = fileIO->getOptionPanelForLoading();
            if(optionPanel){
                optionPanelBox->insertWidget(0, optionPanel);
            }
        }
    }

    if(fileIO->isRegisteredForSingletonItem()){
        fileDialog.setFileMode(QFileDialog::ExistingFile);
    } else {
        fileDialog.setFileMode(QFileDialog::ExistingFiles);
    }
}


void ItemFileIO::Dialog::Impl::onFilterSelected(const QString& filter)
{
    auto filters = fileDialog.nameFilters();
    int index = filters.indexOf(QRegExp(filter, Qt::CaseSensitive, QRegExp::FixedString));
    if(index >= pFileIoList->size()){
        index = 0;
    }
    setTargetFileIO((*pFileIoList)[index]);
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
