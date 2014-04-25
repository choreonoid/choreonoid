/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_GUIBASE_DESCRIPTION_DIALOG_H_INCLUDED
#define CNOID_GUIBASE_DESCRIPTION_DIALOG_H_INCLUDED

#include "Dialog.h"
#include <QPlainTextEdit>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT DescriptionDialog : public Dialog
{
public:
    DescriptionDialog();
        
    void setDescription(const QString& text);

private:
    QPlainTextEdit textEdit;
};
}

#endif
