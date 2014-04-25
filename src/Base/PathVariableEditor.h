/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_PATH_VARIABLE_EDITOR_H_INCLUDED
#define CNOID_BASE_PATH_VARIABLE_EDITOR_H_INCLUDED

#include <QDialog>
#include <QTableWidget>
#include <QLineEdit>

namespace cnoid {
    
class ExtensionManager;

class PathVariableEditor : public QDialog
{
public:
    static void initialize(ExtensionManager* ext);

private:
    QTableWidget* tableWidget;
    QLineEdit* newVariableEntry;

    PathVariableEditor();
    void initAndShow();
    void readPathVariablesFromArchive();
    void appendPathVariable(const QString& name, const QString& paths);
    void writePathVariablesToArchive();
    void onAppendActivated();
};
}

#endif
