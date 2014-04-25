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
    connect(this, SIGNAL(accepted()), this, SLOT(onSigAccepted()));
    connect(this, SIGNAL(finished(int)), this, SLOT(onSigFinished(int)));
    connect(this, SIGNAL(rejected()), this, SLOT(onSigRejected()));
}


void Dialog::onAccepted()
{

}


void Dialog::onRejected()
{

}


void Dialog::onSigAccepted()
{
    onAccepted();
    sigAccepted_();
}


void Dialog::onSigFinished(int result)
{
    sigFinished_(result);
}


void Dialog::onSigRejected()
{
    onRejected();
    sigRejected_();
}
