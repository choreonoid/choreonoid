/**
   @author Shin'ichiro Nakaoka
*/

#include "Dialog.h"
#include "MainWindow.h"

using namespace cnoid;

/**
   This constructor sets MainWindwo::instance() to the parent widget of this dialog
*/
Dialog::Dialog()
    : QDialog(MainWindow::instance())
{
#if defined(__APPLE__) && defined(__MACH__)
    // This is needed to show the dialog over the main window on OS X.
    setWindowFlags(windowFlags() | Qt::Tool);
#endif

    initialize();
}


Dialog::Dialog(QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f)
{
    initialize();
}


void Dialog::initialize()
{
    connect(this, (void(QDialog::*)()) &QDialog::accepted,
            [this](){ onAccepted(); sigAccepted_(); });
    connect(this, (void(QDialog::*)(int)) &QDialog::finished,
            [this](int result){ sigFinished_(result); });
    connect(this, (void(QDialog::*)()) &QDialog::rejected,
            [this](){ onRejected(); sigRejected_(); });

    isWindowPositionKeepingMode_ = false;
}


void Dialog::onAccepted()
{

}


void Dialog::onRejected()
{

}


void Dialog::setWindowPositionKeepingMode(bool on)
{
    isWindowPositionKeepingMode_ = on;
}


void Dialog::show()
{
    QDialog::show();

    if(isWindowPositionKeepingMode_ && !lastWindowPosition_.isNull()){
        setGeometry(lastWindowPosition_);
    }
}


void Dialog::hideEvent(QHideEvent* event)
{
    lastWindowPosition_ = geometry();
    QDialog::hideEvent(event);
}
