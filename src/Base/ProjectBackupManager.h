#ifndef CNOID_BASE_PROJECT_BACKUP_MANAGER_H
#define CNOID_BASE_PROJECT_BACKUP_MANAGER_H

#include <filesystem>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class Item;

class CNOID_EXPORT ProjectBackupManager
{
public:
    static void initializeClass();
    static ProjectBackupManager* instance();
    static bool isAutoBackupEnabled();

    void enableAutoBackupAtStartup();
    void setAutoBackupInterval(int sec);
    void setMaxNumBackups(int n);
    bool checkForPreviousCrash();
    void enableAutoBackup(bool on = true);
    void disableAutoBackup();
    bool saveProjectAsBackup();
    bool getItemBackupFileInformation(
        Item* item,
        std::filesystem::path& out_backupFilePath,
        std::filesystem::path& out_hardLinkFilePath,
        std::string& out_fileFormat);
    void setItemBackupFileFormat(Item* item, const std::string& fileFormat);
    const std::vector<std::filesystem::path>& getBackupProjectFilePaths();
    bool restoreProject(int backupProjectIndex);
    void showConfigDialog();
    void showRecoveryDialog();

    class Impl;

private:
    ProjectBackupManager();
    ~ProjectBackupManager();

    Impl* impl;
};

}

#endif
