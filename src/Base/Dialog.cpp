/**
   @author Shin'ichiro Nakaoka
*/

#include "Dialog.h"
#include "MainWindow.h"
#include <QStyle>
#include <QKeyEvent>

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
    connect(this, (void(QDialog::*)(int)) &QDialog::finished,
            [this](int result){ onFinished(result); sigFinished_(result); });
    connect(this, (void(QDialog::*)()) &QDialog::accepted,
            [this](){ onAccepted(); sigAccepted_(); });
    connect(this, (void(QDialog::*)()) &QDialog::rejected,
            [this](){ onRejected(); sigRejected_(); });

    isWindowPositionKeepingMode_ = false;
    isEnterKeyClosePreventionMode_ = false;
}


void Dialog::onFinished(int /* result */)
{

}


void Dialog::onAccepted()
{

}


void Dialog::onRejected()
{

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


void Dialog::keyPressEvent(QKeyEvent* event)
{
    if(isEnterKeyClosePreventionMode_){
        // Prevent the dialog from closing when the enter key is pressed on a child widget
        int key = event->key();
        if(key == Qt::Key_Return || key == Qt::Key_Enter){
            return;
        }
    }
            
    QDialog::keyPressEvent(event);
}


int Dialog::layoutHorizontalSpacing()
{
    return style()->pixelMetric(QStyle::PM_LayoutHorizontalSpacing);
}


int Dialog::layoutVerticalSpacing()
{
    return style()->pixelMetric(QStyle::PM_LayoutVerticalSpacing);
}
