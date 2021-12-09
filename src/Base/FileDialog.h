#ifndef CNOID_BASE_FILE_DIALOG_H
#define CNOID_BASE_FILE_DIALOG_H

#include <cnoid/Signal>
#include <QFileDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT FileDialog : public QDialog
{
public:
    // For the customization from the executable
    static void setShareDirectoryPresetEnabled(bool on);
    
    FileDialog();
    FileDialog(QWidget* parent, Qt::WindowFlags f = 0);
    ~FileDialog();

    void updatePresetDirectories();
    bool selectFilePath(const std::string& filePath);
    void insertOptionPanel(QWidget* panel);

    SignalProxy<void(int index)> sigFilterSelected();
    SignalProxy<bool(int result), LogicalProduct> sigAboutToFinish();
    [[deprecated("Use sigAboutToFinish")]]
    SignalProxy<bool(int result), LogicalProduct> sigAboutToFinished(){
        return sigAboutToFinish();
    }

    virtual int exec() override;
    
    // Internal QFileDialog object
    QFileDialog* fileDialog();

    // Functions delegated to the internal QFileDialog object
    QDir directory() const;
    QStringList nameFilters() const;
    QStringList selectedFiles() const;
    void selectFile(const QString& filename);
    void selectFile(const std::string& filename);
    void setAcceptMode(QFileDialog::AcceptMode mode);
    void setDirectory(const QString& directory);
    void setDirectory(const std::string& directory);
    void setFileMode(QFileDialog::FileMode mode);
    void setLabelText(QFileDialog::DialogLabel label, const QString& text);
    void setNameFilter(const QString& filter);
    void setNameFilters(const QStringList& filters);
    void setOption(QFileDialog::Option option, bool on = true);
    void setViewMode(QFileDialog::ViewMode mode);

    // Util functions
    void selectNameFilter(int index);
    
private:
    class Impl;
    Impl* impl;
};


}

#endif
