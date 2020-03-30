#ifndef CNOID_BASE_FILE_DIALOG_H
#define CNOID_BASE_FILE_DIALOG_H

#include <QFileDialog>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT FileDialog : public QFileDialog
{
public:
    FileDialog();
    FileDialog(QWidget* parent, Qt::WindowFlags f = 0);

    void updatePresetDirectories();

private:
    void onAccepted();
};

}

#endif
