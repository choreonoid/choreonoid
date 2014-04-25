/**
   @author Shin'ichiro Nakaoka
*/

#include "DescriptionDialog.h"
#include "MainWindow.h"
#include "Separator.h"
#include <QBoxLayout>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QFontMetrics>
#include "gettext.h"

using namespace cnoid;

DescriptionDialog::DescriptionDialog()
    : Dialog(MainWindow::instance())
{
    QVBoxLayout* vbox = new QVBoxLayout();
    
    textEdit.setFrameShape(QFrame::NoFrame);
    textEdit.setReadOnly(true);

    vbox->addWidget(&textEdit);

    vbox->addWidget(new HSeparator());

    QPushButton* okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    vbox->addWidget(buttonBox);
    
    setLayout(vbox);

    QFontMetrics metrics(font());
    resize(metrics.averageCharWidth() * 70, metrics.height() * 24);
}


void DescriptionDialog::setDescription(const QString& text)
{
    textEdit.setPlainText(text);
}
