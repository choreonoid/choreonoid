#ifndef CNOID_BASE_FILE_DIALOG_H
#define CNOID_BASE_FILE_DIALOG_H

#include <cnoid/Signal>
#include <QFileDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT FileDialog : public QDialog
{
public:
    FileDialog(QWidget* parent, Qt::WindowFlags f = 0);
    ~FileDialog();

    virtual int exec() override;
    
    void updatePresetDirectories();
    void insertOptionPanel(QWidget* panel);
    SignalProxy<bool(int result), LogicalProduct> sigAboutToFinished();

    QFileDialog* fileDialog();
    QDir directory() const;
    QStringList nameFilters() const;
    QStringList selectedFiles() const;
    void selectFile(const QString &filename);
    void setAcceptMode(QFileDialog::AcceptMode mode);
    void setDirectory(const QString &directory);
    void setFileMode(QFileDialog::FileMode mode);
    void setLabelText(QFileDialog::DialogLabel label, const QString &text);
    void setNameFilter(const QString &filter);
    void setNameFilters(const QStringList &filters);
    void setOption(QFileDialog::Option option, bool on = true);
    void setViewMode(QFileDialog::ViewMode mode);
    
private:
    class Impl;
    Impl* impl;
};


}

#endif
