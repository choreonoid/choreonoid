#ifndef CNOID_BODY_PLUGIN_BODY_LIBRARY_SELECTION_DIALOG_H
#define CNOID_BODY_PLUGIN_BODY_LIBRARY_SELECTION_DIALOG_H

#include <cnoid/Dialog>
#include "BodyLibraryView.h"
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class CheckBox;

class CNOID_EXPORT BodyLibrarySelectionDialog : public Dialog
{
public:
    BodyLibrarySelectionDialog(QWidget* parent);
    void setAcceptButtonLabel(const char* label);
    CheckBox* checkBoxToStoreFile();
    void updateToLatestBodyLibraryContents(BodyLibraryView::ElementPtr rootElement);
    void setFunctionOnAccepted(std::function<void(BodyLibraryView::ElementPtr group)> func);

protected:
    BodyLibrarySelectionDialog();
    ~BodyLibrarySelectionDialog();
    
private:
    class Impl;
    Impl* impl;
};

}

#endif
