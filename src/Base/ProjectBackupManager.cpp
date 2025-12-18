#include "ProjectBackupManager.h"
#include "App.h"
#include "AppConfig.h"
#include "ProjectManager.h"
#include "RootItem.h"
#include "TimeBar.h"
#include "InfoBar.h"
#include "Timer.h"
#include "Dialog.h"
#include <QLabel>
#include "Buttons.h"
#include "CheckBox.h"
#include "SpinBox.h"
#include "Separator.h"
#include <cnoid/ValueTree>
#include <cnoid/MessageOut>
#include <cnoid/MessageView>
#include <cnoid/Format>
#include <cnoid/UTF8>
#include <QBoxLayout>
#include <QDialogButtonBox>
#include <QTableWidget>
#include <QHeaderView>
#include <fstream>
#include <regex>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <ctime>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

static ProjectBackupManager* instance_ = nullptr;
MappingPtr config;

class ItemFileInfo
{
public:
    filesystem::path filePath;
    string fileFormat;
    int fileConsistencyId;
    ScopedConnection itemConnection;
};

class ConfigDialog : public Dialog
{
public:
    ProjectBackupManager::Impl* manager;
    CheckBox enableAutoBackupCheck;
    SpinBox intervalSpin;
    SpinBox maxNumBackupsSpin;
    CheckBox pauseAutoBackupDuringPlaybackCheck;
    CheckBox pauseAutoBackupDuringContinuousUpdateCheck;
    CheckBox doBackupBeforeStartingContinuousUpdateCheck;
    
    ConfigDialog(ProjectBackupManager::Impl* manager);
    void updateWidgets();
    void show();
    virtual void accept() override;
    virtual void done(int r) override;
};

class RecoveryDialog : public Dialog
{
public:
    ProjectBackupManager::Impl* manager;
    QTableWidget* table;

    RecoveryDialog(ProjectBackupManager::Impl* manager);
    void updateProjectsInTableWidget();
    void show();
    void restoreProject(int projectIndex);
    virtual void reject() override;
};

}

namespace cnoid {

class ProjectBackupManager::Impl
{
public:
    ProjectBackupManager* self;
    ProjectManager* projectManager;
    RootItem* rootItem;
    Timer* timer;
    int interval; // sec
    bool isAutoBackupEnabled;
    bool isAutoBackupPausedDuringPlayback;
    bool isAutoBackupPausedDuringContinuousUpdate;
    bool doBackupBeforeStartingContinuousUpdate;
    int maxNumBackups;
    int autoBackupPauseCounter;

    filesystem::path backupTopDirPath;
    filesystem::path crashMarkerFilePath;
    filesystem::path currentBackupDirPath;
    deque<filesystem::path> backupDirPaths;
    bool isBackupDirectoryEnsured;
    bool isCrashMarkerCreated;

    typedef unordered_map<Item*, ItemFileInfo> ItemFileInfoMap;
    ItemFileInfoMap itemFileInfoMap;

    vector<filesystem::path> backupProjectFilePaths;

    ScopedConnection rootItemConnection;
    
    ConfigDialog* configDialog;
    RecoveryDialog* recoveryDialog;

    Impl(ProjectBackupManager* self);
    ~Impl();
    void writeConfigurations();
    void setAutoBackupInterval(int sec);
    void enableAutoBackup(bool on);
    void pauseAutoBackup();
    void unpauseAutoBackup();
    void onAutoBackupRequest();
    void onTreeContinuousUpdateStateExistenceChanged(bool on);
    bool ensureBackupDirectory();
    bool saveProjectAsBackup();
    void removeOldBackups();
    bool getItemBackupFileInformation(
        Item* item,
        std::filesystem::path& out_backupFilePath,
        std::filesystem::path& out_hardLinkFilePath,
        std::string& out_fileFormat);
    void updateBackupProjectFilePaths();
    bool restoreProject(int backupProjectIndex);
};

}


void ProjectBackupManager::initializeClass()
{
    config = AppConfig::archive()->openMapping("ProjectBackupManager");
    if(config->get("enable_auto_backup", false)){
        App::sigExecutionStarted().connect(
            []{ instance()->enableAutoBackupAtStartup(); });
    }
}


ProjectBackupManager* ProjectBackupManager::instance()
{
    static bool isAboutToQuit = false;

    if(!instance_ && !isAboutToQuit){
        instance_ = new ProjectBackupManager;
        App::sigAboutToQuit().connect(
            []{
                delete instance_;
                instance_ = nullptr;
                isAboutToQuit = true;
            });
    }
    
    return instance_;
}


bool ProjectBackupManager::isAutoBackupEnabled()
{
    if(instance_){
        return instance_->impl->isAutoBackupEnabled;
    }
    return false;
}


ProjectBackupManager::ProjectBackupManager()
{
    impl = new Impl(this);
}


ProjectBackupManager::Impl::Impl(ProjectBackupManager* self)
    : self(self)
{
    projectManager = ProjectManager::instance();
    rootItem = RootItem::instance();
    timer = nullptr;
    interval = config->get("backup_interval", 60); // Default is 1 min.
    isAutoBackupEnabled = false;
    isAutoBackupPausedDuringPlayback = config->get("pause_backup_in_playback", true);
    isAutoBackupPausedDuringContinuousUpdate = config->get("pause_backup_in_continuous_update", true);
    doBackupBeforeStartingContinuousUpdate = config->get("backup_before_continuous_update", true);
    maxNumBackups = config->get("max_backups", 10);
    autoBackupPauseCounter = 0;
    backupTopDirPath = AppConfig::configDataDirPath() / "backup";
    crashMarkerFilePath = backupTopDirPath / "crash_marker.tmp";
    isBackupDirectoryEnsured = false;
    isCrashMarkerCreated = false;
    configDialog = nullptr;
    recoveryDialog = nullptr;
}


ProjectBackupManager::~ProjectBackupManager()
{
    delete impl;
}


ProjectBackupManager::Impl::~Impl()
{
    if(configDialog){
        delete configDialog;
        configDialog = nullptr;
    }
    if(recoveryDialog){
        delete recoveryDialog;
        recoveryDialog = nullptr;
    }
    if(timer){
        delete timer;
        timer = nullptr;
    }
    if(isCrashMarkerCreated){
        filesystem::remove(crashMarkerFilePath);
    }
}


void ProjectBackupManager::Impl::writeConfigurations()
{
    config->write("enable_auto_backup", isAutoBackupEnabled);
    config->write("backup_interval", interval);
    config->write("max_backups", maxNumBackups);
    config->write("pause_backup_in_playback", isAutoBackupPausedDuringPlayback);
    config->write("pause_backup_in_continuous_update", isAutoBackupPausedDuringContinuousUpdate);
    config->write("backup_before_continuous_update", doBackupBeforeStartingContinuousUpdate);
    AppConfig::flush();
}


void ProjectBackupManager::enableAutoBackupAtStartup()
{
    if(checkForPreviousCrash()){
        bool ok = showWarningDialog(
            _("The application has crashed, and auto-backup project files are available for project recovery. "
              "Do you want to show the dialog for recovery?"),
            true);
        if(ok){
            impl->isAutoBackupEnabled = true;
            showRecoveryDialog();
        }
    }
    enableAutoBackup();
}


void ProjectBackupManager::setAutoBackupInterval(int sec)
{
    impl->setAutoBackupInterval(sec);
}


void ProjectBackupManager::Impl::setAutoBackupInterval(int sec)
{
    interval = sec;
    
    if(timer && timer->isActive()){
        enableAutoBackup(true);
    }
}


void ProjectBackupManager::setMaxNumBackups(int n)
{
    impl->maxNumBackups = n;
    impl->removeOldBackups();
}


bool ProjectBackupManager::checkForPreviousCrash()
{
    if(!impl->isCrashMarkerCreated){
        if(filesystem::exists(impl->crashMarkerFilePath)){
            return true;
        }
    }
    return false;
}


void ProjectBackupManager::enableAutoBackup(bool on)
{
    impl->enableAutoBackup(on);
}


void ProjectBackupManager::Impl::enableAutoBackup(bool on)
{
    if(on){
        ensureBackupDirectory();

        if(!timer){
            timer = new Timer;
            timer->sigTimeout().connect(
                [this]{ onAutoBackupRequest(); });
        }
        timer->setInterval(interval * 1000); // to msec
        isAutoBackupEnabled = true;

        if(!rootItemConnection.connected()){
            rootItemConnection =
                rootItem->sigTreeContinuousUpdateStateExistenceChanged().connect(
                    [this](bool on){ onTreeContinuousUpdateStateExistenceChanged(on); });
        }
        
        if(autoBackupPauseCounter == 0){
            timer->start();
        }
        
    } else {
        rootItemConnection.disconnect();
        
        if(timer){
            timer->stop();
        }
        
        isAutoBackupEnabled = false;
    }
}


void ProjectBackupManager::disableAutoBackup()
{
    impl->enableAutoBackup(false);
}


void ProjectBackupManager::Impl::pauseAutoBackup()
{
    ++autoBackupPauseCounter;
    if(timer){
        timer->stop();
    }
}    


void ProjectBackupManager::Impl::unpauseAutoBackup()
{
    if(autoBackupPauseCounter > 0){
        --autoBackupPauseCounter;
        if(isAutoBackupEnabled && autoBackupPauseCounter == 0){
            if(timer && !timer->isActive()){
                enableAutoBackup(true);
            }
        }
    }
}    


void ProjectBackupManager::Impl::onAutoBackupRequest()
{
    if(isAutoBackupPausedDuringPlayback){
        if(TimeBar::instance()->isPlaybackProcessing()){
            return; // Skip auto backup
        }
    }
    if(isAutoBackupPausedDuringContinuousUpdate){
        if(rootItem->hasAnyItemsInContinuousUpdateState()){
            return; // Skip auto backup
        }
    }
    
    saveProjectAsBackup();
}


void ProjectBackupManager::Impl::onTreeContinuousUpdateStateExistenceChanged(bool on)
{
    if(on){
        if(doBackupBeforeStartingContinuousUpdate){
            saveProjectAsBackup();
        }
    }
}


bool ProjectBackupManager::Impl::ensureBackupDirectory()
{
    if(isBackupDirectoryEnsured){
        return true;
    }
        
    bool isDirectoryReady = false;
    std::error_code ec;
    if(filesystem::exists(backupTopDirPath)){
        if(filesystem::is_directory(backupTopDirPath)){
            isDirectoryReady = true;
        } else {
            filesystem::remove(backupTopDirPath, ec);
        }
    }

    if(!isDirectoryReady && !ec){
        isDirectoryReady = filesystem::create_directories(backupTopDirPath, ec);
    }

    if(!isDirectoryReady || ec){
        MessageOut::master()->putErrorln(
            formatR(_("The project backup directory \"{0}\" cannot be created: {1}"),
                    toUTF8(backupTopDirPath.string()), toUTF8(ec.message())));
        return false;
    }

    // Create a crash marker file
    std::ofstream(crashMarkerFilePath.string()).close();
    isCrashMarkerCreated = true;
        
    backupDirPaths.clear();
    static std::regex dateTimePattern("^\\d{4}-\\d{2}-\\d{2}-\\d{2}-\\d{2}-\\d{2}-.+$");
    for(const auto& entry : filesystem::directory_iterator(backupTopDirPath, ec)){
        if(filesystem::is_directory(entry)){
            auto& path = entry.path();
            std::string dirName = path.filename().string();
            if(std::regex_match(dirName, dateTimePattern)){
                backupDirPaths.push_back(path);
            }
        }
    }
    if(!backupDirPaths.empty()){
        std::sort(backupDirPaths.begin(), backupDirPaths.end(),
                  [](const filesystem::path& a, const filesystem::path& b){
                      return a.string() > b.string();
                  });
    }

    isBackupDirectoryEnsured = true;

    return true;
}


bool ProjectBackupManager::saveProjectAsBackup()
{
    return impl->saveProjectAsBackup();
}


bool ProjectBackupManager::Impl::saveProjectAsBackup()
{
    if(!ensureBackupDirectory()){
        return false;
    }
    
    auto projectName = projectManager->currentProjectName();
    if(projectName.empty()){
        projectName = "NewProject";
    }

    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    char datetime[20];
    std::strftime(datetime, sizeof(datetime), "%Y-%m-%d-%H-%M-%S", localTime);
    string dirName = formatC("{0}-{1}", datetime, projectName);
    
    currentBackupDirPath = backupTopDirPath / fromUTF8(dirName);

    if(filesystem::exists(currentBackupDirPath)){
        currentBackupDirPath.clear();
        return false; // Skip this time. Another Choreonoid instance may be doing backup.
    }

    std::error_code ec;
    bool isDirectoryReady = filesystem::create_directories(currentBackupDirPath, ec);

    if(!isDirectoryReady || ec){
        MessageOut::master()->putErrorln(
            formatR(_("The project backup directory \"{0}\" cannot be created: {1}"),
                    toUTF8(currentBackupDirPath.string()), toUTF8(ec.message())));
        currentBackupDirPath.clear();
        return false;
    }

    auto backupProjectFilePath = currentBackupDirPath / (fromUTF8(projectName) + ".cnoid");
    auto backupProjectFile = toUTF8(backupProjectFilePath.string());

    auto infoBar = InfoBar::instance();
    infoBar->notify(
        formatR(_("Automatically saving project \"{0}\" to \"{1}\" as backup ..."),
                projectName, backupProjectFile));

    projectManager->saveProjectAsBackup(backupProjectFile);

    infoBar->notify(
        formatR(_("Automatically saving project \"{0}\" to \"{1}\" as backup ... done."),
                projectName, backupProjectFile));

    backupDirPaths.push_front(currentBackupDirPath);
    currentBackupDirPath.clear();

    removeOldBackups();

    return true;
}


void ProjectBackupManager::Impl::removeOldBackups()
{
    while(backupDirPaths.size() > maxNumBackups){
        std::error_code ec;
        filesystem::remove_all(backupDirPaths.back(), ec);
        backupDirPaths.pop_back();
    }
}


bool ProjectBackupManager::getItemBackupFileInformation
(Item* item,
 std::filesystem::path& out_backupFilePath,
 std::filesystem::path& out_hardLinkFilePath,
 std::string& out_fileFormat)
{
    return impl->getItemBackupFileInformation(
        item, out_backupFilePath, out_hardLinkFilePath, out_fileFormat);
}


bool ProjectBackupManager::Impl::getItemBackupFileInformation
(Item* item,
 std::filesystem::path& out_backupFilePath,
 std::filesystem::path& out_hardLinkFilePath,
 std::string& out_fileFormat)
{
    if(currentBackupDirPath.empty()){
        return false;
    }

    // Note: filename is in native character encoding
    auto filename = fromUTF8(item->fileName());
    if(filename.empty()){
        filename = fromUTF8(item->name());
    }
    if(filename.empty()){
        filename = "item";
    }

    out_fileFormat = item->fileFormat();
    
    out_backupFilePath = currentBackupDirPath / filename;

    static std::regex pattern("^([^-.]*)(-([0-9]+))?(\\.(.*?))?$");
    std::smatch matches;
    while(filesystem::exists(out_backupFilePath)){
        if(!std::regex_match(filename, matches, pattern)){
            return false;
        }
        int number = 2;
        if(matches[3].matched){
            number = std::stoi(matches[3].str()) + 1;
        }
        if(matches[5].matched){ // extension
            filename = formatR("{0}-{1}.{2}", matches[1].str(), number, matches[5].str());
        } else {
            filename = formatR("{0}-{1}", matches[1].str(), number);
        }
        out_backupFilePath = currentBackupDirPath / filename;
    }

    int lastFileConsistencyId;
    auto& info = itemFileInfoMap[item];
    if(info.itemConnection.connected()){
        lastFileConsistencyId = info.fileConsistencyId;
        info.fileConsistencyId = item->fileConsistencyId();
    } else {
        info.itemConnection =
            item->sigDisconnectedFromRoot().connect(
                [this, item]{ itemFileInfoMap.erase(item); });
        info.fileConsistencyId = item->fileConsistencyId();
        lastFileConsistencyId = info.fileConsistencyId - 1;
    }

    out_hardLinkFilePath.clear();
    if(item->isConsistentWithFile()){
        const auto& itemFilePath = item->filePath();
        if(!itemFilePath.empty()){
            out_hardLinkFilePath = fromUTF8(itemFilePath);
            if(!filesystem::exists(out_hardLinkFilePath)){
                out_hardLinkFilePath.clear();
            }
        }
    }
    if(out_hardLinkFilePath.empty()){
        if(item->fileConsistencyId() == lastFileConsistencyId){
            if(filesystem::exists(info.filePath)){
                out_hardLinkFilePath = info.filePath;
                out_fileFormat = info.fileFormat;
            }
        }
    }

    info.filePath = out_backupFilePath;

    return true;
}


void ProjectBackupManager::setItemBackupFileFormat(Item* item, const std::string& fileFormat)
{
    auto it = impl->itemFileInfoMap.find(item);
    if(it != impl->itemFileInfoMap.end()){
        it->second.fileFormat = fileFormat;
    }
}


const std::vector<std::filesystem::path>& ProjectBackupManager::getBackupProjectFilePaths()
{
    impl->updateBackupProjectFilePaths();
    return impl->backupProjectFilePaths;
}


void ProjectBackupManager::Impl::updateBackupProjectFilePaths()
{
    ensureBackupDirectory();
    
    backupProjectFilePaths.clear();

    static std::regex dirNamePattern("^(\\d{4}-\\d{2}-\\d{2}-\\d{2}-\\d{2}-\\d{2})-(.+)$");
    std::smatch matches;
    for(auto& dirPath : backupDirPaths){
        string dirName = dirPath.filename().string();
        if(std::regex_match(dirName, matches, dirNamePattern)){
            string projectName = matches[2].str();
            auto projectFilePath = dirPath / (projectName + ".cnoid");
            backupProjectFilePaths.push_back(projectFilePath);
        }
    }
}


bool ProjectBackupManager::restoreProject(int backupProjectIndex)
{
    return impl->restoreProject(backupProjectIndex);
}


bool ProjectBackupManager::Impl::restoreProject(int backupProjectIndex)
{
    bool restored = false;
    
    if(backupProjectIndex < static_cast<int>(backupProjectFilePaths.size())){
        projectManager->clearProject();
        auto& path = backupProjectFilePaths[backupProjectIndex];
        auto items = projectManager->loadProject(toUTF8(path.string()));
        if(!items.empty()){
            restored = true;
        }
    }
        
    return restored;
}


void ProjectBackupManager::showConfigDialog()
{
    if(!impl->configDialog){
        impl->configDialog = new ConfigDialog(impl);
    }
    impl->configDialog->show();
}


void ProjectBackupManager::showRecoveryDialog()
{
    if(!impl->recoveryDialog){
        impl->recoveryDialog = new RecoveryDialog(impl);
    }
    impl->pauseAutoBackup();
    impl->projectManager->tryToCloseProject();
    impl->recoveryDialog->show();
    impl->unpauseAutoBackup();
}


ConfigDialog::ConfigDialog(ProjectBackupManager::Impl* manager_)
    : manager(manager_)
{
    setWindowTitle(_("Project Auto-Backup Configuration"));

    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    enableAutoBackupCheck.setText(_("Enable auto backup"));
    vbox->addWidget(&enableAutoBackupCheck);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Time interval")));
    intervalSpin.setRange(1, 9999);
    hbox->addWidget(&intervalSpin);
    hbox->addWidget(new QLabel(_("sec.")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Max number of backups")));
    maxNumBackupsSpin.setRange(1, 1000);
    hbox->addWidget(&maxNumBackupsSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    pauseAutoBackupDuringPlaybackCheck.setText(_("Pause auto backup during playback"));
    vbox->addWidget(&pauseAutoBackupDuringPlaybackCheck);

    pauseAutoBackupDuringContinuousUpdateCheck.setText(_("Pause auto backup during continuous update"));
    vbox->addWidget(&pauseAutoBackupDuringContinuousUpdateCheck);

    doBackupBeforeStartingContinuousUpdateCheck.setText(_("Backup before starting continuous update"));
    vbox->addWidget(&doBackupBeforeStartingContinuousUpdateCheck);

    int vspace = layoutVerticalSpacing();
    vbox->addSpacing(vspace);
    vbox->addWidget(new HSeparator);
    vbox->addSpacing(vspace);

    auto buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(QDialogButtonBox::Ok);
    buttonBox->addButton(QDialogButtonBox::Cancel);
    connect(buttonBox, SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));
    vbox->addWidget(buttonBox);
}


void ConfigDialog::updateWidgets()
{
    enableAutoBackupCheck.setChecked(manager->isAutoBackupEnabled);
    intervalSpin.setValue(manager->interval);
    maxNumBackupsSpin.setValue(manager->maxNumBackups);
    pauseAutoBackupDuringPlaybackCheck.setChecked(manager->isAutoBackupPausedDuringPlayback);
    pauseAutoBackupDuringContinuousUpdateCheck.setChecked(manager->isAutoBackupPausedDuringContinuousUpdate);
    doBackupBeforeStartingContinuousUpdateCheck.setChecked(manager->doBackupBeforeStartingContinuousUpdate);
}


void ConfigDialog::show()
{
    manager->pauseAutoBackup();
    updateWidgets();
    Dialog::show();
}


void ConfigDialog::accept()
{
    manager->setAutoBackupInterval(intervalSpin.value());
    manager->self->setMaxNumBackups(maxNumBackupsSpin.value());
    manager->isAutoBackupPausedDuringPlayback = pauseAutoBackupDuringPlaybackCheck.isChecked();
    manager->isAutoBackupPausedDuringContinuousUpdate = pauseAutoBackupDuringContinuousUpdateCheck.isChecked();
    manager->doBackupBeforeStartingContinuousUpdate = doBackupBeforeStartingContinuousUpdateCheck.isChecked();
    manager->enableAutoBackup(enableAutoBackupCheck.isChecked());
    manager->writeConfigurations();

    Dialog::accept();
}


void ConfigDialog::done(int r)
{
    manager->unpauseAutoBackup();
    Dialog::done(r);
}



RecoveryDialog::RecoveryDialog(ProjectBackupManager::Impl* manager)
    : manager(manager)
{
    setWindowTitle(_("Backup Project Recovery"));

    auto vbox = new QVBoxLayout;
    setLayout(vbox);

    table = new QTableWidget(this);
    table->setColumnCount(2);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->verticalHeader()->hide();
    table->setHorizontalHeaderItem(0, new QTableWidgetItem(_("Date and Time")));
    table->setHorizontalHeaderItem(1, new QTableWidgetItem(_("Project")));
    auto header = table->horizontalHeader();
    header->setSectionResizeMode(QHeaderView::ResizeToContents);
    header->setStretchLastSection(true);
    
    vbox->addWidget(table);

    connect(table, &QTableWidget::cellDoubleClicked,
            [this](int row, int column){ restoreProject(row); });

    auto buttonBox = new QDialogButtonBox(this);
    auto recoveryButton = new PushButton(_("&Recovery"));
    recoveryButton->setDefault(true);

    recoveryButton->sigClicked().connect(
        [this]{
            auto items = table->selectedItems();
            if(!items.empty()){
                restoreProject(items[0]->row());
            }
        });
            
    buttonBox->addButton(recoveryButton, QDialogButtonBox::ActionRole);

    buttonBox->addButton(QDialogButtonBox::Close);
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(reject()));

    vbox->addWidget(buttonBox);
}


void RecoveryDialog::updateProjectsInTableWidget()
{
    static std::regex dirNamePattern("^(\\d{4}-\\d{2}-\\d{2}-\\d{2}-\\d{2}-\\d{2})-(.+)$");
    std::smatch matches;
    auto& projectFilePaths = manager->self->getBackupProjectFilePaths();
    table->setRowCount(projectFilePaths.size());
    for(size_t i=0; i < projectFilePaths.size(); ++i){
        auto& path = projectFilePaths[i];
        auto directory = path.parent_path().filename().string();
        if(std::regex_match(directory, matches, dirNamePattern)){
            string date = matches[1].str();
            string projectName = toUTF8(matches[2].str());
            auto dateItem = new QTableWidgetItem(date.c_str());
            dateItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            table->setItem(i, 0, dateItem);
            auto nameItem = new QTableWidgetItem(projectName.c_str());
            nameItem->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
            table->setItem(i, 1, nameItem);
        }
    }
}


void RecoveryDialog::show()
{
    manager->pauseAutoBackup();
    
    updateProjectsInTableWidget();

    int h = table->rowHeight(0) * 14;
    int w = table->columnWidth(0) * 3;
    resize(w, h);
    
    Dialog::show();
}


void RecoveryDialog::restoreProject(int projectIndex)
{
    if(!manager->restoreProject(projectIndex)){
        showErrorDialog(_("Selected backup is not found."));
        updateProjectsInTableWidget();
    }
}


void RecoveryDialog::reject()
{
    manager->unpauseAutoBackup();
    Dialog::reject();
}
