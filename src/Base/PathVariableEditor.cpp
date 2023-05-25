#include "PathVariableEditor.h"
#include "AppConfig.h"
#include "MainWindow.h"
#include "ExtensionManager.h"
#include "Buttons.h"
#include <cnoid/FilePathVariableProcessor>
#include <cnoid/ValueTree>
#include <cnoid/UTF8>
#include <cnoid/stdx/filesystem>
#include <QBoxLayout>
#include <QLabel>
#include <QHeaderView>
#include <QDialogButtonBox>
#include <map>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

class PathVariableItem : public QTableWidgetItem
{
public:
    QString path;
        
    PathVariableItem(const QString& path)
        : path(path) {
        setFlags(Qt::ItemIsEnabled|Qt::ItemIsEditable);
    }

    virtual QVariant data(int role) const {

        if(role == Qt::DisplayRole || role == Qt::EditRole){
            return path;
        }
        return QTableWidgetItem::data(role);
    }

    virtual void setData(int role, const QVariant& value) {
        if(role == Qt::EditRole && value.type() == QVariant::String){
            path = value.toString();
        } else {
            QTableWidgetItem::setData(role, value);
        }
    }
};

}


PathVariableEditor* PathVariableEditor::instance()
{
    static PathVariableEditor* instance_ = new PathVariableEditor;
    return instance_;
}


PathVariableEditor::PathVariableEditor()
    : QDialog(MainWindow::instance())
{
    setWindowTitle(_("Path Variables"));

    QVBoxLayout* vbox = new QVBoxLayout();
    tableWidget = new QTableWidget(this);
    tableWidget->setColumnCount(2);
    tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    tableWidget->setSelectionMode(QAbstractItemView::NoSelection);

    tableWidget->setHorizontalHeaderItem(0, new QTableWidgetItem(_("Variable")));
    tableWidget->setHorizontalHeaderItem(1, new QTableWidgetItem(_("Path")));
    tableWidget->horizontalHeader()->setStretchLastSection(true);

    tableWidget->verticalHeader()->hide();
    tableWidget->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    
    vbox->addWidget(tableWidget, 1);

    QHBoxLayout* hbox = new QHBoxLayout();
    QLabel* label = new QLabel(_("Variable"));
    hbox->addWidget(label);
    newVariableEntry = new QLineEdit();
    hbox->addWidget(newVariableEntry);
    PushButton* button = new PushButton(_("Append"));
    hbox->addWidget(button);
    button->sigClicked().connect(std::bind(&PathVariableEditor::onAppendActivated, this));
    vbox->addLayout(hbox);
    
    QPushButton* createButton = new QPushButton(_("&Apply"));
    createButton->setDefault(true);
    QPushButton* cancelButton = new QPushButton(_("&Cancel"));
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(createButton, QDialogButtonBox::AcceptRole);
    buttonBox->addButton(cancelButton, QDialogButtonBox::RejectRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    connect(buttonBox,SIGNAL(rejected()), this, SLOT(reject()));
    vbox->addWidget(buttonBox);

    setLayout(vbox);
}


void PathVariableEditor::show()
{
    readPathVariablesFromArchive();
    if(exec() == QDialog::Accepted){
        writePathVariablesToArchive();
    }
}


void PathVariableEditor::readPathVariablesFromArchive()
{
    auto archive = AppConfig::archive();
    MappingPtr pathVars = archive->openMapping("path_variables");
    tableWidget->setRowCount(0);

    if(pathVars->empty()){
        auto oldPathVars = archive->findMapping("pathVariables");
        if(oldPathVars->isValid()){
            pathVars->insert(oldPathVars);
            archive->remove("pathVariables");
        }
    }

    for(Mapping::const_iterator p = pathVars->begin(); p != pathVars->end(); ++p){
        if(p->second->isListing()){
            const std::string& name = p->first;
            if(!name.empty()){
                Listing* paths = p->second->toListing();
                for(int i=0; i < paths->size(); ++i){
                    if(paths->at(i)->isString()){
                        appendPathVariable(name.c_str(), paths->at(i)->toString().c_str());
                    }
                }
            }
        }
    }
}


void PathVariableEditor::appendPathVariable(const QString& name, const QString& path)
{
    int row = tableWidget->rowCount();
    tableWidget->setRowCount(row + 1);

    QTableWidgetItem* nameItem = new QTableWidgetItem(name);
    nameItem->setFlags(Qt::ItemIsEnabled);
    tableWidget->setItem(row, 0, nameItem);

    PathVariableItem* pathSetItem = new PathVariableItem(path);
    tableWidget->setItem(row, 1, pathSetItem);
}


void PathVariableEditor::writePathVariablesToArchive()
{
    MappingPtr pathVars = AppConfig::archive()->openMapping("path_variables");
    pathVars->clear();

    int n = tableWidget->rowCount();
    for(int i=0; i < n; ++i){
        PathVariableItem* item = dynamic_cast<PathVariableItem*>(tableWidget->item(i, 1));
        if(item){
            string name = tableWidget->item(i, 0)->text().toStdString();
            if(!name.empty() && !item->path.isEmpty()){
                Listing* listing = pathVars->openListing(name);
                stdx::filesystem::path path(fromUTF8(item->path.toStdString()));
                listing->append(toUTF8(path.generic_string()), DOUBLE_QUOTED);
            }
        }
    }
    if(pathVars->empty()){
        AppConfig::archive()->remove("path_variables");
    }

    FilePathVariableProcessor::systemInstance()->restoreUserVariables(pathVars);
}


void PathVariableEditor::onAppendActivated()
{
    appendPathVariable(newVariableEntry->text(), "");
}
