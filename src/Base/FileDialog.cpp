#include "FileDialog.h"
#include "AppConfig.h"
#include "ProjectManager.h"
#include <cnoid/ExecutablePath>

using namespace std;
using namespace cnoid;

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


FileDialog::FileDialog()
    : FileDialog(nullptr)
{

}


FileDialog::FileDialog(QWidget* parent, Qt::WindowFlags f)
    : QFileDialog(parent, f)
{
    setOption(QFileDialog::DontUseNativeDialog);

    QObject::connect(this, &FileDialog::accepted, [&](){ onAccepted(); });
}


void FileDialog::updatePresetDirectories()
{
    QList<QUrl> urls;

    if(DisableToClearDefaultSettings){
        urls = sidebarUrls();
    } else {
        urls << QUrl("file:");
        urls << QUrl::fromLocalFile(QDir::homePath());
    }
    
    urls << QUrl::fromLocalFile(shareDirectory().c_str());
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


void FileDialog::onAccepted()
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
