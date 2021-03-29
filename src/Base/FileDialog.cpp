#include "FileDialog.h"
#include "AppConfig.h"
#include "ProjectManager.h"
#include "MainWindow.h"
#include <cnoid/ExecutablePath>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <cnoid/stdx/optional>
#include <QBoxLayout>
#include <QStyle>

using namespace std;
using namespace cnoid;
namespace filesystem = stdx::filesystem;

namespace {

/**
   QFileDialog is implemented to store settings including its access history, last
   accessed directory, sidebar urls, etc. automatically. If you try to change those
   settings and store them by yourself, the settings conflict with those sotred automatically.
   The automatic setting storing cannot be disabled at runtime, and Qt application may expect
   shareding the settings. So you have to be careful about chaning the settings by yourself.
   If the following variable is set to false, FileDialog does not clear the default settings
   to keep it after using Choreonoid.
*/
constexpr bool DisableToClearDefaultSettings = false;

constexpr int MaxHistorySize = 8;

bool isBeforeChoosingAnyFile = true;

}

namespace cnoid {

class FileDialog::Impl : public QFileDialog
{
public:
    FileDialog* self;
    QBoxLayout* optionPanelBox;
    stdx::optional<Signal<void(int index)>> sigFilterSelected;
    Signal<bool(int result), LogicalProduct> sigAboutToFinished;
    
    Impl(FileDialog* self);
    void updatePresetDirectories();    
    void onFilterSelected(const QString& selected);
    void onFinished(int result);
    void storeRecentDirectories();
};

}


FileDialog::FileDialog()
    : FileDialog(MainWindow::instance())
{

}


FileDialog::FileDialog(QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    impl = new Impl(this);
}


FileDialog::Impl::Impl(FileDialog* self)
    : self(self)
{
    setWindowFlags(windowFlags() & ~Qt::Dialog);
    setOption(QFileDialog::DontUseNativeDialog);
    setSizeGripEnabled(false);
    self->setSizeGripEnabled(true);
        
    auto vbox = new QVBoxLayout;
    vbox->setSpacing(0);
    self->setLayout(vbox);

    vbox->addWidget(this);

    optionPanelBox = new QHBoxLayout;
    auto sty = self->style();
    int left = sty->pixelMetric(QStyle::PM_LayoutLeftMargin);
    int right = sty->pixelMetric(QStyle::PM_LayoutRightMargin);
    int bottom = sty->pixelMetric(QStyle::PM_LayoutBottomMargin);
    optionPanelBox->setContentsMargins(left, 0, right, bottom);
    optionPanelBox->addStretch();
    vbox->addLayout(optionPanelBox);

    QObject::connect(this, &QFileDialog::finished,
                     [this](int result){ onFinished(result); });

    //QObject::connect(this, &QFileDialog::accepted, [&](){ storeRecentDirectories(); });
}


FileDialog::~FileDialog()
{
    delete impl;
}


void FileDialog::updatePresetDirectories()
{
    impl->updatePresetDirectories();
}


void FileDialog::Impl::updatePresetDirectories()
{
    QList<QUrl> urls;

    if(DisableToClearDefaultSettings){
        urls = sidebarUrls();
    } else {
        urls << QUrl("file:");
        urls << QUrl::fromLocalFile(QDir::homePath());
    }
    
    urls << QUrl::fromLocalFile(shareDir().c_str());
    auto projectDir = ProjectManager::instance()->currentProjectDirectory();

    if(!projectDir.empty()){
        urls << QUrl::fromLocalFile(projectDir.c_str());
    }
    urls << QUrl::fromLocalFile(QDir::currentPath());

    setSidebarUrls(urls);

    if(!DisableToClearDefaultSettings){
        QStringList qhistory;
        auto& recentDirs = *AppConfig::archive()->findListing("file_dialog_recent_dirs");
        if(recentDirs.isValid() && !recentDirs.empty()){
            for(int i = recentDirs.size() - 1; i >= 0; --i){
                if(recentDirs[i].isString()){
                    qhistory << recentDirs[i].toString().c_str();
                }
            }
        }
        setHistory(qhistory);
    }

    bool directoryDetermined = false;
    if(!isBeforeChoosingAnyFile){
        auto qhistory = history();
        if(!qhistory.empty()){
            setDirectory(qhistory.last());
            directoryDetermined = true;
        }
    }
    if(!directoryDetermined){
        auto projectDir = ProjectManager::instance()->currentProjectDirectory();
        if(!projectDir.empty()){
            setDirectory(projectDir.c_str());
        } else {
            setDirectory(QDir::current());
        }
    } 
}


bool FileDialog::selectFilePath(const std::string& filePath)
{
    bool selected = false;
    if(!filePath.empty()){
        filesystem::path path(fromUTF8(filePath));
        filesystem::path dir(path.parent_path());
        if(filesystem::exists(dir)){
            setDirectory(toUTF8(dir.string()));
            if(filesystem::exists(path)){
                selectFile(toUTF8(path.filename().string()));
                selected = true;
            }
        }
    }
    return selected;
}


void FileDialog::insertOptionPanel(QWidget* panel)
{
    impl->optionPanelBox->insertWidget(0, panel);
}


SignalProxy<void(int index)> FileDialog::sigFilterSelected()
{
    if(!impl->sigFilterSelected){
        stdx::emplace(impl->sigFilterSelected);
        QObject::connect(impl, &QFileDialog::filterSelected,
                [this](const QString& filter){ impl->onFilterSelected(filter); });
    }
    return *impl->sigFilterSelected;
}


void FileDialog::Impl::onFilterSelected(const QString& selected)
{
    auto filters = nameFilters();
    for(int index = 0; index < filters.size(); ++index){
        if(filters[index] == selected){
            (*sigFilterSelected)(index);
            break;
        }
    }
}


void FileDialog::selectNameFilter(int index)
{
    auto filters = impl->nameFilters();
    if(index < filters.size()){
        impl->selectNameFilter(filters[index]);
    }
}


SignalProxy<bool(int result), LogicalProduct> FileDialog::sigAboutToFinished()
{
    return impl->sigAboutToFinished;
}


int FileDialog::exec()
{
    impl->show();
    return QDialog::exec();
}
    

void FileDialog::Impl::onFinished(int result)
{
    if(sigAboutToFinished(result)){
        if(result == QFileDialog::Accepted){
            storeRecentDirectories();
        }
        self->done(result);
    }
}


void FileDialog::Impl::storeRecentDirectories()
{
    isBeforeChoosingAnyFile = false;

    if(!DisableToClearDefaultSettings){
        QDir latestDir = directory();
        ListingPtr recentDirs = new Listing;
        recentDirs->append(latestDir.absolutePath().toStdString());
        auto oldRecentDirs = AppConfig::archive()->openListing("file_dialog_recent_dirs");
        for(auto& node : *oldRecentDirs){
            if(node->isString()){
                auto dir = node->toString();
                if(QDir(dir.c_str()) != latestDir){
                    recentDirs->append(dir);
                    if(recentDirs->size() >= MaxHistorySize){
                        break;
                    }
                }
            }
        }
        AppConfig::archive()->insert("file_dialog_recent_dirs", recentDirs);
    }
}


QFileDialog* FileDialog::fileDialog()
{
    return impl;
}


QDir FileDialog::directory() const
{
    return impl->directory();
}


QStringList FileDialog::nameFilters() const
{
    return impl->nameFilters();
}


QStringList FileDialog::selectedFiles() const
{
    return impl->selectedFiles();
}


void FileDialog::selectFile(const QString& filename)
{
    impl->selectFile(filename);
}


void FileDialog::selectFile(const std::string& filename)
{
    impl->selectFile(filename.c_str());
}


void FileDialog::setAcceptMode(QFileDialog::AcceptMode mode)
{
    impl->setAcceptMode(mode);
}


void FileDialog::setDirectory(const QString& directory)
{
    impl->setDirectory(directory);
}


void FileDialog::setDirectory(const std::string& directory)
{
    impl->setDirectory(directory.c_str());
}


void FileDialog::setFileMode(QFileDialog::FileMode mode)
{
    impl->setFileMode(mode);
}


void FileDialog::setLabelText(QFileDialog::DialogLabel label, const QString &text)
{
    impl->setLabelText(label, text);
}


void FileDialog::setNameFilter(const QString& filter)
{
    impl->setNameFilter(filter);
}
                               

void FileDialog::setNameFilters(const QStringList& filters)
{
    impl->setNameFilters(filters);
}


void FileDialog::setOption(QFileDialog::Option option, bool on)
{
    return impl->setOption(option, on);
}


void FileDialog::setViewMode(QFileDialog::ViewMode mode)
{
    impl->setViewMode(mode);
}
