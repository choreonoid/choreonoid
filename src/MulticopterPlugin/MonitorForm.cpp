/**
   @author Japan Atomic Energy Agency
*/

#include "MonitorForm.h"
#include "ui_MonitorForm.h"

using namespace std;

MonitorForm::MonitorForm(QWidget *parent) : QWidget(parent), ui(new Ui::MonitorForm)
{
    ui->setupUi(this);
}

MonitorForm::~MonitorForm()
{
    delete ui;
}

void
MonitorForm::write(const string& msg)
{
    ui->textEdit->append(QString::fromStdString(msg));
}

void
MonitorForm::writeln(const string& msg)
{
    ui->textEdit->append(QString::fromStdString(msg));
}

void
MonitorForm::clear()
{
    ui->textEdit->clear();
}
