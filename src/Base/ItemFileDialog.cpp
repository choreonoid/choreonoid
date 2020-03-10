#include "ItemFileDialog.h"
#include "ItemFileIOImpl.h"
#include "AppConfig.h"
#include "RootItem.h"
#include "MainWindow.h"
#include "MessageView.h"
#include <cnoid/FileUtil>
#include <cnoid/ExecutablePath>
#include <QFileDialog>
#include <QBoxLayout>
#include <QStyle>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
namespace filesystem = cnoid::stdx::filesystem;
using fmt::format;


namespace cnoid {

class ItemFileDialog::Impl
{
    ItemFileDialog* self;
    QFileDialog fileDialog;
    const vector<ItemFileIO*>* pFileIoList;
    ItemFileIO* targetFileIO;
    QWidget* optionPanel;
    QBoxLayout* optionPanelBox;
    bool isSingletonItem;
    
public:
    Impl(ItemFileDialog* self);
    ItemList<Item>  loadItems(
        const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem);

private:
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


ItemList<Item> ItemFileDialog::loadItems
(const std::vector<ItemFileIO*>& fileIoList, Item* parentItem, bool doAddition, Item* nextItem)
{
    return impl->loadItems(fileIoList, parentItem, doAddition, nextItem);
}


ItemList<Item> ItemFileDialog::Impl::loadItems
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


void ItemFileDialog::Impl::setTargetFileIO(ItemFileIO* fileIO)
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


void ItemFileDialog::Impl::onFilterSelected(const QString& filter)
{
    auto filters = fileDialog.nameFilters();
    int index = filters.indexOf(QRegExp(filter, Qt::CaseSensitive, QRegExp::FixedString));
    if(index >= pFileIoList->size()){
        index = 0;
    }
    setTargetFileIO((*pFileIoList)[index]);
}
