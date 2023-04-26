#ifndef CNOID_BASE_PATH_VARIABLE_EDITOR_H
#define CNOID_BASE_PATH_VARIABLE_EDITOR_H

#include <QDialog>
#include <QTableWidget>
#include <QLineEdit>

namespace cnoid {
    
class PathVariableEditor : public QDialog
{
public:
    static PathVariableEditor* instance();
    void show();

private:
    QTableWidget* tableWidget;
    QLineEdit* newVariableEntry;

    PathVariableEditor();
    void readPathVariablesFromArchive();
    void appendPathVariable(const QString& name, const QString& paths);
    void writePathVariablesToArchive();
    void onAppendActivated();
};

}

#endif
