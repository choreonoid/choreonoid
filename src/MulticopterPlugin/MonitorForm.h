/**
   @author Japan Atomic Energy Agency
*/

#pragma once

#include <QWidget>

namespace Ui {
class MonitorForm;
}

class MonitorForm : public QWidget
{
    Q_OBJECT
public:

    explicit MonitorForm(QWidget *parent = 0);

    ~MonitorForm();

    void write(const std::string& msg);
    void writeln(const std::string& msg);
    void clear();
private:
    Ui::MonitorForm* ui;
};
