/**
   @author Shizuko Hattori
*/

#include "TextEditView.h"
#include "TextEdit.h"
#include "MainWindow.h"
#include "ViewManager.h"
#include "ItemTreeView.h"
#include "AbstractTextItem.h"
#include "Buttons.h"
#include "Timer.h"
#include <cnoid/ConnectionSet>
#include <cnoid/stdx/filesystem>
#include <QAction>
#include <QHBoxLayout>
#include <QFile>
#include <QLabel>
#include <QMessageBox>
#include <QTextCodec>
#include <QTextDocumentWriter>
#include <sstream>
#include "gettext.h"

using namespace cnoid;
using namespace std::placeholders;
namespace filesystem = cnoid::stdx::filesystem;

#define FILE_CHECK_TIME 1000   //msec

namespace cnoid {

class TextEditViewImpl
{
public:
    TextEditViewImpl(TextEditView* self);
    ~TextEditViewImpl();
            
    TextEditView* self;
    PlainTextEdit textEdit;
    
private:
    ConnectionSet connections;
    Connection connectionOfCurrentBodyItemDetachedFromRoot;
    AbstractTextItemPtr currentTextItem_;
    ItemList<AbstractTextItem> selectedTextItems_;
    bool viewActive;

    QLabel fileNameLabel;
    QLabel lineLabel;
    Timer timer;
    std::time_t fileTimeStamp;
    uintmax_t fileSize;

    QAction *actionUndo,
        *actionRedo,
        *actionCut,
        *actionCopy,
        *actionPaste;

    void onItemSelectionChanged(const ItemList<AbstractTextItem>& textItems);
    void onTextItemDetachedFromRoot();
    void open();
    void maybeSave();
    void save();
    void setCurrentFileName();
    void timeOut();
    void cursorPositionChanged();
    void onActivated(bool on);
};
}


void TextEditView::initializeClass(ExtensionManager* ext)
{
    ext->viewManager().registerClass<TextEditView>(
        "TextEditView", N_("Text Editor"), ViewManager::SINGLE_OPTIONAL);    
}


TextEditView::TextEditView()
{
    impl = new TextEditViewImpl(this);
}


TextEditView::~TextEditView()
{
    delete impl;
}


TextEditViewImpl::TextEditViewImpl(TextEditView* self)
    : self(self)
{
    self->setDefaultLayoutArea(View::CENTER);

    actionUndo = new QAction(&textEdit);
    actionUndo->setShortcut(QKeySequence::Undo);
    QObject::connect(actionUndo, SIGNAL(triggered()), &textEdit, SLOT(undo()));
    actionRedo = new QAction(&textEdit);
    actionRedo->setShortcut(QKeySequence::Redo);
    QObject::connect(actionRedo, SIGNAL(triggered()), &textEdit, SLOT(redo()));
    actionCut = new QAction(&textEdit);
    actionCut->setShortcut(QKeySequence::Cut);
    QObject::connect(actionCut, SIGNAL(triggered()), &textEdit, SLOT(cut()));
    actionCopy = new QAction(&textEdit);
    actionCopy->setShortcut(QKeySequence::Copy);
    QObject::connect(actionCopy, SIGNAL(triggered()), &textEdit, SLOT(copy()));
    actionPaste = new QAction(&textEdit);
    actionPaste->setShortcut(QKeySequence::Paste);
    QObject::connect(actionPaste, SIGNAL(triggered()), &textEdit, SLOT(paste()));

    QVBoxLayout* vbox = new QVBoxLayout();
    QHBoxLayout* hbox = new QHBoxLayout();
    setCurrentFileName();
    fileNameLabel.setSizePolicy(QSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred));
    hbox->addWidget(&fileNameLabel, 10);
    PushButton* saveButton = new PushButton(_("Save"));
    saveButton->sigClicked().connect(std::bind(&TextEditViewImpl::save, this));
    hbox->addWidget(saveButton);
    vbox->addLayout(hbox);
    vbox->addWidget(&textEdit);
    lineLabel.setAlignment(Qt::AlignBottom | Qt::AlignRight);
    vbox->addWidget(&lineLabel);
    self->setLayout(vbox);

    textEdit.sigCursorPositionChanged().connect(
        std::bind(&TextEditViewImpl::cursorPositionChanged, this));

    connections.add(
        ItemTreeView::mainInstance()->sigSelectionChanged().connect(
            std::bind(&TextEditViewImpl::onItemSelectionChanged, this, _1)));
    timer.setSingleShot(true);
    connections.add(
        timer.sigTimeout().connect(
            std::bind(&TextEditViewImpl::timeOut, this)));

    self->sigActivated().connect(std::bind(&TextEditViewImpl::onActivated, this, true));
    self->sigDeactivated().connect(std::bind(&TextEditViewImpl::onActivated, this ,false));

    viewActive = false;
}


TextEditViewImpl::~TextEditViewImpl()
{
    connections.disconnect();
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}


void TextEditViewImpl::onItemSelectionChanged(const ItemList<AbstractTextItem>& textItems)
{
    if(selectedTextItems_ != textItems)
        selectedTextItems_ = textItems;
    else
        return;

    AbstractTextItemPtr firstItem = textItems.toSingle();

    if(firstItem && firstItem != currentTextItem_){
        if(currentTextItem_)
            maybeSave();
        currentTextItem_ = firstItem;
        connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
        connectionOfCurrentBodyItemDetachedFromRoot = currentTextItem_->sigDetachedFromRoot().connect(
            std::bind(&TextEditViewImpl::onTextItemDetachedFromRoot, this));
        open();
    }
}


void TextEditViewImpl::onTextItemDetachedFromRoot()
{
    maybeSave();
    textEdit.clear();
    currentTextItem_ = 0;
    setCurrentFileName();
    connectionOfCurrentBodyItemDetachedFromRoot.disconnect();
}


void TextEditViewImpl::open()
{
    timer.stop();
    QString fileName(currentTextItem_->textFilename().c_str());
    if (!QFile::exists(fileName))
        return;
    QFile file(fileName);
    if (!file.open(QFile::ReadWrite))
        return;
    
    QByteArray data = file.readAll();
    QString str = QString::fromLocal8Bit(data);
    textEdit.setPlainText(str);  

    setCurrentFileName();
}


void TextEditViewImpl::maybeSave()
{
    if (!textEdit.document()->isModified())
        return;
    timer.stop();
    QMessageBox::StandardButton ret;
    ret = QMessageBox::warning(MainWindow::instance(), _("Warning"),
                               _("The document has been modified.\n Do you want to save your changes?"),
                               QMessageBox::Yes | QMessageBox::No );
    if (ret == QMessageBox::Yes)
        return save();
    else if (ret == QMessageBox::No)
        return;
}


void TextEditViewImpl::save()
{
    timer.stop();
    if(!currentTextItem_)
        return;
    QString fileName(currentTextItem_->textFilename().c_str());
    if(fileName.isEmpty())
        return;
    QTextDocumentWriter writer(fileName);
    writer.setFormat("plaintext");
    bool success = writer.write(textEdit.document());
    if (success)
        setCurrentFileName();
}


void TextEditViewImpl::setCurrentFileName()
{
    if(!currentTextItem_){
        fileNameLabel.setText(_("unselected"));
        textEdit.setReadOnly(true);
        timer.stop();
    }else{
        QString fileName(currentTextItem_->textFilename().c_str());
        fileNameLabel.setText(fileName);
        textEdit.setReadOnly(false);
        textEdit.document()->setModified(false);
        filesystem::path fpath(currentTextItem_->textFilename());
        fileTimeStamp = filesystem::last_write_time_to_time_t(fpath);
        fileSize = filesystem::file_size(fpath);
        if(viewActive)
            timer.start(FILE_CHECK_TIME);
    }
}


void TextEditViewImpl::timeOut()
{
    if(currentTextItem_){
        std::string filename = currentTextItem_->textFilename();
        if(!filename.empty()){
            filesystem::path fpath(filename);
            if(filesystem::exists(fpath)){
                if( filesystem::last_write_time_to_time_t(fpath) != fileTimeStamp ||
                    filesystem::file_size(fpath) != fileSize ){
                    QMessageBox::StandardButton ret;
                    ret = QMessageBox::warning(MainWindow::instance(), _("Warning"),
                                               _("The file has been modified outside of text editor.\nDo you want to reload it?"),
                                               QMessageBox::Yes | QMessageBox::No );
                    if (ret == QMessageBox::Yes){
                        textEdit.clear();
                        open();
                    }else if (ret == QMessageBox::No){
                        fileTimeStamp = filesystem::last_write_time_to_time_t(fpath);
                        fileSize = filesystem::file_size(fpath);
                        textEdit.document()->setModified(true);
                    }
                }
            }
        }
    }
    if(viewActive)
        timer.start(FILE_CHECK_TIME);
}


void TextEditViewImpl::cursorPositionChanged()
{
    QTextCursor cur = textEdit.textCursor();
    std::stringstream s;
    s << cur.blockNumber()+1 << "  :  " << cur.columnNumber()+1 << "  ";
    lineLabel.setText(QString(s.str().c_str()));
}


void TextEditViewImpl::onActivated(bool on)
{
    viewActive = on;

    if(currentTextItem_ && on)
        timer.start(FILE_CHECK_TIME);
    else
        timer.stop();
}
