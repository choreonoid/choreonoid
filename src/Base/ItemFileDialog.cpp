#include "ItemFileDialog.h"
#include "ItemFileIOImpl.h"
#include "FileDialog.h"
#include "RootItem.h"
#include "MainWindow.h"
#include "MessageView.h"
#include <QBoxLayout>
#include <QStyle>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;

namespace cnoid {

class ItemFileDialog::Impl : public FileDialog
{
public:
    ItemFileDialog* self;
    enum Mode { Load, Save };
    Mode mode;
    const vector<ItemFileIO*>* pFileIoList;
    ItemFileIO* targetFileIO;
    ItemPtr currentItemToSave;
    bool isCurrentItemOptionsApplied;
    QWidget* optionPanel;
    QBoxLayout* optionPanelBox;
    bool isSingletonItem;
    bool isExportMode;
    
    Impl(ItemFileDialog* self);
    ItemList<Item>  loadItems(
        const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem);
    bool saveItem(Item* item, const std::vector<ItemFileIO*>& fileIoList);
    bool initializeFileIoFilters(const vector<ItemFileIO*>& fileIoList);
    void setTargetFileIO(ItemFileIO* fileIO);
    void onFilterSelected(const QString& filter);
};

}

ItemFileDialog::ItemFileDialog()
    : QDialog(MainWindow::instance())
{
    impl = new Impl(this);
}


ItemFileDialog::~ItemFileDialog()
{
    delete impl;
}


ItemFileDialog::Impl::Impl(ItemFileDialog* self)
    : FileDialog(self),
      self(self)
{
    pFileIoList = nullptr;
    targetFileIO = nullptr;
    
    self->setSizeGripEnabled(true);
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    self->setLayout(vbox);

    setWindowFlags(windowFlags() & ~Qt::Dialog);
    setSizeGripEnabled(false);
    setViewMode(QFileDialog::List);
    setLabelText(QFileDialog::Reject, _("Cancel"));

    vbox->addWidget(this);

    optionPanel = nullptr;
    optionPanelBox = new QHBoxLayout;
    auto sty = self->style();
    int left = sty->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int right = sty->pixelMetric(QStyle::PM_LayoutRightMargin);
    int bottom = sty->pixelMetric(QStyle::PM_LayoutBottomMargin);
    optionPanelBox->setContentsMargins(left, 0, right, bottom);
    optionPanelBox->addStretch();
    vbox->addLayout(optionPanelBox);

    QObject::connect(this, &QFileDialog::filterSelected,
                     [this](const QString& filter){ onFilterSelected(filter); });

    QObject::connect(this, SIGNAL(finished(int)), self, SLOT(done(int)));

    isExportMode = false;
}


ItemList<Item> ItemFileDialog::loadItems
(const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem)
{
    return impl->loadItems(fileIoList, parentItem, doAddition, nextItem);
}


ItemList<Item> ItemFileDialog::Impl::loadItems
(const vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem)
{
    mode = Load;
    
    ItemList<Item> loadedItems;

    if(!initializeFileIoFilters(fileIoList)){
        return loadedItems;
    }

    bool isImportMode = (fileIoList.front()->interfaceLevel() == ItemFileIO::Conversion);

    if(self->windowTitle().isEmpty()){
        string title;
        if(!isImportMode){
            title = _("Load {0}");
        } else {
            title = _("Import {0}");
        }
        self->setWindowTitle(format(title, fileIoList.front()->caption()).c_str());
    }
    setAcceptMode(QFileDialog::AcceptOpen);

    if(!isImportMode){
        setLabelText(QFileDialog::Accept, _("Load"));
    } else {
        setLabelText(QFileDialog::Accept, _("Import"));
    }

    updatePresetDirectories();

    if(self->exec() == QDialog::Accepted){

        QStringList filenames = selectedFiles();

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
            bool loaded = targetFileIO->impl->loadItem(
                ItemFileIO::Dialog, item, filenames[i].toStdString(), parentItem, doAddition, nextItem, nullptr);
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


void ItemFileDialog::setExportMode(bool on)
{
    impl->isExportMode = on;
}


bool ItemFileDialog::saveItem(Item* item, const std::vector<ItemFileIO*>& fileIoList)
{
    return impl->saveItem(item, fileIoList);
}


bool ItemFileDialog::Impl::saveItem(Item* item, const std::vector<ItemFileIO*>& fileIoList)
{
    mode = Save;
    currentItemToSave = item;
    isCurrentItemOptionsApplied = false;
    
    if(!initializeFileIoFilters(fileIoList)){
        string message;
        if(!isExportMode){
            message = _("Saving {0} to a file is not supported");
        } else {
            message = _("Exporting {0} to a file is not supported");
        }
        MessageView::instance()->putln(format(message, item->name()), MessageView::HIGHLIGHT);
        currentItemToSave.reset();
        return false;
    }


    string itemLabel = format("{0} \"{1}\"", fileIoList.front()->caption(), item->name());
    
    if(self->windowTitle().isEmpty()){
        string title;
        if(!isExportMode){
            title = _("Save {} as");
        } else {
            title = _("Export {} as");
        }
        self->setWindowTitle(format(title, itemLabel).c_str());
    }
    setAcceptMode(QFileDialog::AcceptSave);
    setLabelText(QFileDialog::Accept, _("Save"));
    setFileMode(QFileDialog::AnyFile);

    updatePresetDirectories();

    // default filename
    selectFile(item->name().c_str());

    bool saved = false;
    
    if(self->exec() == QDialog::Accepted){
        auto filenames = selectedFiles();
        if(!filenames.isEmpty()){
            auto filename = filenames.front().toStdString();
            if(optionPanel){
                targetFileIO->fetchOptionPanelForSaving();
            }
            
            // add a lacking extension automatically
            auto exts = targetFileIO->extensions();
            if(!exts.empty()){
                bool hasExtension = false;
                string dotextension = filesystem::path(filename).extension().string();
                if(!dotextension.empty()){
                    string extension = dotextension.substr(1); // remove the first dot
                    if(std::find(exts.begin(), exts.end(), extension) != exts.end()){
                        hasExtension = true;
                    }
                }
                if(!hasExtension && !exts.empty()){
                    filename += ".";
                    filename += exts[0];
                }
            }

            saved = targetFileIO->impl->saveItem(ItemFileIO::Dialog, item, filename, nullptr);
        }
    }
    /*
    else {
        string message;
        if(!isExportMode){
            message = _("Saving {0} was canceled.");
        } else {
            message = _("Exporting {0} was canceled.");
        }
        MessageView::instance()->putln(format(message, itemLabel), MessageView::HIGHLIGHT);
    }
    */

    for(auto& fileIO : fileIoList){
        if(auto panel = fileIO->getOptionPanelForLoading()){
            panel->setParent(nullptr);
        }
    }

    currentItemToSave.reset();

    return saved;
}


bool ItemFileDialog::Impl::initializeFileIoFilters(const vector<ItemFileIO*>& fileIoList)
{
    if(fileIoList.empty()){
        return false;
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
    setNameFilters(filters);

    setTargetFileIO(fileIoList.front());

    return true;
}


void ItemFileDialog::Impl::setTargetFileIO(ItemFileIO* fileIO)
{
    targetFileIO = fileIO;

    if(optionPanel){
        optionPanel->setParent(nullptr);
        optionPanel = nullptr;
    }
    int api = fileIO->api();
    if(api & ItemFileIO::Options){
        if(mode == Save){
            if(!isCurrentItemOptionsApplied &&
               fileIO->isFormat(currentItemToSave->fileFormat())){
                fileIO->resetOptions();
                fileIO->restoreOptions(currentItemToSave->fileOptions());
                isCurrentItemOptionsApplied = true;
            }
        }
        if(api & ItemFileIO::OptionPanelForLoading){
            optionPanel = fileIO->getOptionPanelForLoading();
            if(optionPanel){
                optionPanelBox->insertWidget(0, optionPanel);
            }
        } else if(api & ItemFileIO::OptionPanelForSaving){
            optionPanel = fileIO->getOptionPanelForSaving(currentItemToSave);
            if(optionPanel){
                optionPanelBox->insertWidget(0, optionPanel);
            }
        }
    }

    if(mode == Load){
        if(fileIO->isRegisteredForSingletonItem()){
            setFileMode(QFileDialog::ExistingFile);
        } else {
            setFileMode(QFileDialog::ExistingFiles);
        }
    }
}


void ItemFileDialog::Impl::onFilterSelected(const QString& filter)
{
    auto filters = nameFilters();
    int index = filters.indexOf(QRegExp(filter, Qt::CaseSensitive, QRegExp::FixedString));
    if(index >= pFileIoList->size()){
        index = 0;
    }
    setTargetFileIO((*pFileIoList)[index]);
}
